#!/usr/bin/env python3
"""
webrtc_pusher — 使用单一 asyncio 事件循环 + aiortc 的 WebRTC 视频推流节点
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import asyncio
import threading
import json
import os
import signal

from flask import Flask, request, jsonify
from flask_cors import CORS
from aiortc import RTCPeerConnection, MediaStreamTrack, RTCSessionDescription
from aiortc.contrib.media import MediaRelay

# ════════════════════════════════════════════
# Camera frame holder (thread-safe)
# ════════════════════════════════════════════
class CameraHolder:
    def __init__(self):
        self._frame = None
        self._lock = threading.Lock()

    def update(self, cv_img):
        with self._lock:
            self._frame = cv_img.copy() if cv_img is not None else None

    def get(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

camera_holder = CameraHolder()

# ════════════════════════════════════════════
# ROS2 camera subscriber
# ════════════════════════════════════════════
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("webrtc_camera_node")
        self.bridge = CvBridge()
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("webrtc_port", 8082)
        topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, topic, self.image_callback, 5)
        self.get_logger().info(f"CameraSubscriber ready on {topic}")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            camera_holder.update(cv_img)
        except Exception as e:
            self.get_logger().warn(f"Image decode error: {e}")

# ════════════════════════════════════════════
# WebRTC video track
# ════════════════════════════════════════════
class RobotCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.counter = 0

    async def recv(self):
        import av
        pts, time_base = await self.next_timestamp()
        frame = camera_holder.get() or np.zeros((480, 640, 3), dtype=np.uint8)
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        # 浏览器只接受 YUV420p 编码的 RTP 负载；aiortc 不会自动从 BGR 转 YUV
        video_frame = video_frame.reformat(format="yuv420p")
        video_frame.pts = pts
        video_frame.time_base = time_base
        self.counter += 1
        return video_frame

from aiortc.mediastreams import VideoFrame

# ════════════════════════════════════════════
# Shared asyncio loop in background thread
# ════════════════════════════════════════════
_loop = asyncio.new_event_loop()
_pcs: set[RTCPeerConnection] = set()


def _start_bg_loop():
    asyncio.set_event_loop(_loop)
    _loop.run_forever()


bg_thread = threading.Thread(target=_start_bg_loop, daemon=True)
bg_thread.start()


async def _create_answer(offer_sdp: str) -> dict:
    """WebRTC answer creation — runs on the shared bg loop."""
    pc = RTCPeerConnection()
    _pcs.add(pc)

    pc.addTrack(RobotCameraTrack())

    @pc.on("iceconnectionstatechange")
    async def on_ice_state():
        if pc.iceConnectionState in ("failed", "closed", "disconnected"):
            _pcs.discard(pc)
            await pc.close()

    await pc.setRemoteDescription(RTCSessionDescription(sdp=offer_sdp, type="offer"))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}


def _cleanup_zombie_pcs():
    """Remove closed/failed connections from the set."""
    for pc in list(_pcs):
        if pc.iceConnectionState in ("failed", "closed", "disconnected"):
            _pcs.discard(pc)


# ════════════════════════════════════════════
# Flask signaling (synchronous → schedules onto bg loop)
# ════════════════════════════════════════════
app = Flask(__name__)
CORS(app)

@app.route("/offer", methods=["POST"])
def offer():
    params = request.json
    if not params:
        return jsonify({"error": "missing JSON body"}), 400
    offer_sdp = params.get("sdp", "")
    if not offer_sdp:
        return jsonify({"error": "missing sdp field"}), 400

    future = asyncio.run_coroutine_threadsafe(
        _create_answer(offer_sdp), _loop
    )
    try:
        result = future.result(timeout=10)
        return jsonify(result)
    except asyncio.TimeoutError:
        return jsonify({"error": "WebRTC negotiation timeout"}), 504
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/health", methods=["GET"])
def health():
    _cleanup_zombie_pcs()
    return jsonify({"status": "ok", "connections": len(_pcs)})


def run_flask(host="0.0.0.0", port=8082):
    """Start Flask in a daemon thread."""
    app.run(host=host, port=port, debug=False, use_reloader=False)

# ════════════════════════════════════════════
# Main
# ════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    port = node.get_parameter("webrtc_port").value

    node.get_logger().info(f"WebRTC pusher — https://{os.uname()[1]}:{port}/offer")

    flask_thread = threading.Thread(
        target=run_flask, args=("0.0.0.0", port), daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        for pc in list(_pcs):
            asyncio.run_coroutine_threadsafe(pc.close(), _loop).result(timeout=3)
        _loop.call_soon_threadsafe(_loop.stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
