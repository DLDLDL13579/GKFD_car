#!/usr/bin/env python3
"""
ws_mqtt_proxy — WebSocket ↔ MQTT 桥接代理
功能：浏览器端通过 WebSocket 连到本机端口，数据透明转发到 MQTT Broker
"""
import rclpy
from rclpy.node import Node
import asyncio
import websockets
import socket
import threading

MQTT_BROKER = "192.168.1.67"
MQTT_PORT = 1883
WS_PORT = 9001


async def proxy(ws):
    """单个 WebSocket 连接 ↔ MQTT TCP 双向转发"""
    loop = asyncio.get_event_loop()
    reader, writer = None, None

    try:
        # 连接 MQTT broker (TCP)
        reader, writer = await asyncio.open_connection(MQTT_BROKER, MQTT_PORT)

        async def ws_to_mqtt():
            """WebSocket → MQTT"""
            async for msg in ws:
                if isinstance(msg, bytes):
                    writer.write(msg)
                else:
                    writer.write(msg.encode())
                await writer.drain()

        async def mqtt_to_ws():
            """MQTT → WebSocket"""
            while True:
                data = await reader.read(65536)
                if not data:
                    break
                await ws.send(data)

        # 并发运行双向转发
        await asyncio.gather(ws_to_mqtt(), mqtt_to_ws())

    except Exception as e:
        pass  # 连接断开是正常行为
    finally:
        if writer:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass


async def run_server():
    async with websockets.serve(proxy, "0.0.0.0", WS_PORT):
        await asyncio.Future()  # 永远运行


class WsProxyNode(Node):
    def __init__(self):
        super().__init__("ws_mqtt_proxy")
        self.declare_parameter("ws_port", 9001)
        self.declare_parameter("mqtt_host", "192.168.1.67")
        self.declare_parameter("mqtt_port", 1883)

        global WS_PORT, MQTT_BROKER, MQTT_PORT
        WS_PORT = self.get_parameter("ws_port").value
        MQTT_BROKER = self.get_parameter("mqtt_host").value
        MQTT_PORT = self.get_parameter("mqtt_port").value

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"WS↔MQTT proxy started: ws://0.0.0.0:{WS_PORT} <-> "
            f"{MQTT_BROKER}:{MQTT_PORT}"
        )

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(run_server())

    def shutdown(self):
        self._loop.call_soon_threadsafe(self._loop.stop)


def main(args=None):
    rclpy.init(args=args)
    node = WsProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
