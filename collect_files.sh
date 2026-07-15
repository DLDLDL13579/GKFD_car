#!/bin/bash

# 输出文件名
OUTPUT_FILE="combined_files.txt"

# 清空或创建输出文件
> "$OUTPUT_FILE"

# 使用绝对路径定义文件列表
FILES=(
    "/home/nvidia/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py"
    "/home/nvidia/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/wheeltec_lidar.launch.py"
    "/home/nvidia/wheeltec_ros2/src/largemodel/launch/largemodel_control.launch.py"
    "/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab/params/rtabmap_nav_params.yaml"
    "/home/nvidia/wheeltec_ros2/src/largemodel/launch/largemodel_nav.launch.py"
    "/home/nvidia/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch.py"
    "/home/nvidia/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/base_serial.launch.py"
    "/home/nvidia/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/wheeltec_ekf.launch.py"
    "/home/nvidia/wheeltec_ros2/src/turn_on_wheeltec_robot/config/imu.yaml"
    "/home/nvidia/wheeltec_ros2/src/turn_on_wheeltec_robot/config/ekf.yaml"
    "/home/nvidia/wheeltec_ros2/src/largemodel/launch/largemodel_slam.launch.py"
    "/home/nvidia/wheeltec_ros2/src/largemodel/largemodel/action_service.py"
    "/home/nvidia/wheeltec_ros2/src/largemodel/largemodel/model_service.py"
    "/home/nvidia/wheeltec_ros2/src/largemodel/largemodel/utils/promot.py"
    "/home/nvidia/wheeltec_ros2/src/largemodel/config/model_config.yaml"


)

# 遍历每个文件
for file in "${FILES[@]}"; do
    echo "===== $file =====" >> "$OUTPUT_FILE"
    if [ -f "$file" ]; then
        cat "$file" >> "$OUTPUT_FILE"
    else
        echo "错误：文件不存在 - $file" >> "$OUTPUT_FILE"
    fi
    echo -e "\n\n" >> "$OUTPUT_FILE"
done

echo "完成！结果已保存到 $OUTPUT_FILE"