import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ollama_ros_msgs.srv import Chat
import re
import ast
import json
import os

class LLMNavManager(Node):
    def __init__(self):
        super().__init__('llm_nav_manager')
        
        self.client = self.create_client(Chat, '/chat_service')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ 等待大模型服务 /chat_service 上线...')

    def load_dynamic_locations(self):
        # 动态加载坐标库，请确保路径正确
        file_path = '/home/nvidia/wheeltec_ros2/src/ollama_ros_chat/locations.json'
        if os.path.exists(file_path):
            with open(file_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        return {}

    def send_command(self, user_text):
        locations = self.load_dynamic_locations()
        if not locations:
             self.get_logger().error("❌ 未找到坐标库，请检查 locations.json 文件！")
             return

        # 动态生成 prompt
        loc_str = ", ".join([f"{name}=[{coords[0]}, {coords[1]}]" for name, coords in locations.items()])
        
        request = Chat.Request()
        request.content = f"系统指令：你现在是ROS2导航控制器。根据用户的话，提取目标地点并输出预设坐标。不要任何思考解释，严格只输出格式为 [x, y] 的坐标。预设坐标库：[{loc_str}]。 用户的话：“{user_text}”。请输出坐标："
        
        self.get_logger().info(f"🧠 当前加载地图点位: {list(locations.keys())}")
        self.get_logger().info(f"🗣️ 听到指令: '{user_text}'，大模型正在思考...")
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response_text = future.result().content
            self.get_logger().info(f"💡 原始回答: {response_text}")
            self.parse_and_navigate(response_text)
        else:
            self.get_logger().error("❌ 未能联系上大模型。")

    def parse_and_navigate(self, response_text):
        match = re.search(r'\[(.*?)\]', response_text)
        if match:
            try:
                coords = ast.literal_eval(f"[{match.group(1)}]")
                x, y = float(coords[0]), float(coords[1])
                self.publish_goal(x, y)
            except Exception as e:
                self.get_logger().error(f"❌ 坐标解析失败: {e}")
        else:
            self.get_logger().error("❌ 未能从大模型回复中提取到有效坐标！")

    def publish_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0 # 默认朝向
        
        self.goal_pub.publish(msg)
        self.get_logger().info(f"🚀 成功下发导航目标: X={x}, Y={y}。小车出发！")

def main():
    rclpy.init()
    node = LLMNavManager()
    
    node.get_logger().info("🤖 脑机接口导航助手已启动！(输入 'q' 退出)")
    
    try:
        while rclpy.ok():
            user_input = input("\n👉 请输入你的指令 (例如: 去厨房 / 我困了): ")
            
            if user_input.strip().lower() in ['q', 'quit', 'exit']:
                node.get_logger().info("👋 退出导航助手...")
                break
                
            if not user_input.strip():
                continue
                
            node.send_command(user_input)
            
    except KeyboardInterrupt:
        node.get_logger().info("\n手动中断程序...")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()