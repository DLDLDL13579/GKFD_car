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
            self._frame_count = getattr(self, "_frame_count", 0) + 1
            if self._frame_count % 30 == 1:
                print(f"[webrtc_cam] recv frame #{self._frame_count} shape={cv_img.shape}", flush=True)
        except Exception as e:
            print(f"[webrtc_cam] decode error: {e}", flush=True)

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
        import traceback
        pts, time_base = await self.next_timestamp()
        bgr = camera_holder.get()
        if bgr is None:
            bgr = np.zeros((480, 640, 3), dtype=np.uint8)
            is_dummy = True
        else:
            is_dummy = False
        try:
            # 先转 RGB24（av 处理 RGB 比 BGR 稳定），再 reformat 到 YUV420p
            rgb = bgr[..., ::-1].copy()  # BGR -> RGB
            video_frame = VideoFrame.from_ndarray(rgb, format="rgb24")
            video_frame = video_frame.reformat(format="yuv420p")
            video_frame.pts = pts
            video_frame.time_base = time_base
            self.counter += 1
            if self.counter % 30 == 1:
                print(f"[webrtc_track] frame #{self.counter} shape={rgb.shape} dummy={is_dummy}", flush=True)
            return video_frame
        except Exception as e:
            print(f"[webrtc_track] frame error: {e}\n{traceback.format_exc()}", flush=True)
            # 推一个最简 yuv420p 黑帧保活
            black = np.zeros((480, 640, 3), dtype=np.uint8)
            vf = VideoFrame.from_ndarray(black[..., ::-1], format="rgb24").reformat(format="yuv420p")
            vf.pts = pts
            vf.time_base = time_base
            return vf

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
    """WebRTC answer creation — runs on the shared bg loop.
    配置 STUN 让跨网段也能穿透；不自动 close PC，靠 /health 周期清理僵尸。"""
    # 用 Google 公开 STUN + 自己的 host candidate（局域网）
    pc = RTCPeerConnection(iceServers=[
        {"urls": "stun:stun.l.google.com:19302"},
        {"urls": "stun:stun1.l.google.com:19302"},
    ])
    _pcs.add(pc)
    track = RobotCameraTrack()
    pc.addTrack(track)

    @pc.on("connectionstatechange")
    async def on_conn_state():
        print(f"[pc] connectionState={pc.connectionState}", flush=True)

    @pc.on("iceconnectionstatechange")
    async def on_ice_state():
        print(f"[pc] iceConnectionState={pc.iceConnectionState}", flush=True)

    @pc.on("icegatheringstatechange")
    async def on_ice_gather():
        print(f"[pc] iceGatheringState={pc.iceGatheringState}", flush=True)

    @pc.on("track")
    async def on_track(track):
        print(f"[pc] track kind={track.kind} id={track.id}", flush=True)

    print(f"[pc] created, setRemoteDescription...", flush=True)
    await pc.setRemoteDescription(RTCSessionDescription(sdp=offer_sdp, type="offer"))
    print(f"[pc] createAnswer...", flush=True)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    print(f"[pc] answer ready, type={pc.localDescription.type}", flush=True)
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
