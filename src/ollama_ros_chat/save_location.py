import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import json
import os
import threading

class LocationSaver(Node):
    def __init__(self):
        super().__init__('location_saver')
        
        # 1. 建立 TF 坐标监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. 指向大模型的记忆库文件
        self.file_path = '/home/nvidia/wheeltec_ros2/src/ollama_ros_chat/locations.json'
        
        self.get_logger().info('📡 坐标测绘雷达已开启，准备接收你的标记...')

    def save_current_location(self, location_name):
        try:
            # 向系统查询：当前 base_footprint 在 map 中的实时位置
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                now, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 提取 XY 坐标，并保留两位小数
            x = round(trans.transform.translation.x, 2)
            y = round(trans.transform.translation.y, 2)
            
            self.update_json(location_name, x, y)
            
        except TransformException as ex:
            self.get_logger().error(f'❌ 无法获取坐标，请确认小车是否已在地图中定位！报错: {ex}')

    def update_json(self, name, x, y):
        data = {}
        
        # 如果文件存在，先读取里面原有的记忆
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r', encoding='utf-8') as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    pass
        
        # 写入或覆盖新坐标
        data[name] = [x, y]
        
        # 重新保存回 JSON 文件
        with open(self.file_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
            
        self.get_logger().info(f'🎉 成功记录新地点！ "{name}" -> [{x}, {y}]')
        self.get_logger().info('🧠 大模型已同步学会该地点！')

def main():
    rclpy.init()
    node = LocationSaver()
    
    # 开启一个后台线程让 ROS 节点保持运行（为了持续监听坐标）
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print("\n" + "="*50)
    print("🗺️  小车自动测绘记录仪")
    print("使用方法：用手柄/键盘把小车开到指定位置，然后输入地名。")
    print("输入 'q' 退出程序。")
    print("="*50 + "\n")
    
    try:
        while rclpy.ok():
            user_input = input("📍 小车已就位，这里是哪里？请输入地名 (例如: 冰箱): ")
            
            if user_input.strip().lower() in ['q', 'quit', 'exit']:
                print("👋 退出测绘模式...")
                break
                
            if not user_input.strip():
                continue
                
            node.save_current_location(user_input.strip())
            
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()