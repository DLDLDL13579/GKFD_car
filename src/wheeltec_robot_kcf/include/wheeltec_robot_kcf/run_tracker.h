#ifndef TRANSBOT_ASTRA_RUN_TRACKER_H
#define TRANSBOT_ASTRA_RUN_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include "kcftracker.h"
#include "PID.h"
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include <time.h>

using namespace std;
using namespace cv;
using std::placeholders::_1;

// ================= 新增：引入 run_tracker.cpp 中的全局变量 =================
// 因为原来的鼠标框选逻辑定义在 run_tracker.cpp 中的全局变量里，
// 这里通过 extern 声明，允许我们在下面构造函数中直接修改它们，从而实现“代码自动框选”
extern cv::Rect selectRect; // 追踪框的坐标和尺寸
extern bool bRenewROI;      // 触发 KCF 初始化追踪的标志位
// =========================================================================

class ImageConverter : public rclcpp::Node {
    // 声明 ROS 2 的发布者和订阅者指针
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;   // 发布画好框的图像
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;   // 发布小车运动速度指令
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // 订阅彩色图像 (找目标)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_; // 订阅深度图像 (测距离)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Joy_sub_;       // (预留) 手柄控制订阅
    
public:
    ImageConverter() : Node("image_converter")
    {
        // --- 1. 定义 PID 与控制相关参数的默认值 ---
        float linear_KP = 3.0;
        float linear_KI = 0.0;
        float linear_KD = 1.0;
        float angular_KP = 1.5;
        float angular_KI = 0.0;
        float angular_KD = 1.0;
        float targetDist = 1.0; // 默认距离目标 1.0 米
        bool refresh = false;
            
        // --- 2. 声明 ROS 2 参数 ---
        // 允许在终端通过 --ros-args -p 的方式修改这些参数
        this->declare_parameter<float>("linear_KP_", 3.0);
        this->declare_parameter<float>("linear_KI_", 0.0);
        this->declare_parameter<float>("linear_KD_", 1.0);
        this->declare_parameter<float>("angular_KP_", 0.5);
        this->declare_parameter<float>("angular_KI_", 0.0);
        this->declare_parameter<float>("angular_KD_", 2.0);
        this->declare_parameter<float>("targetDist_", 1.0);
        this->declare_parameter<bool>("refresh_", false);

        // ================= 新增：声明大模型坐标参数 =================
        // 声明接收大模型 (Python服务端) 传递过来的框选对角坐标 (x1, y1) 和 (x2, y2)
        this->declare_parameter<int>("x1", 0);
        this->declare_parameter<int>("y1", 0);
        this->declare_parameter<int>("x2", 0);
        this->declare_parameter<int>("y2", 0);
        // =========================================================
         
        // --- 3. 获取 ROS 2 参数的值 ---
        this->get_parameter<float>("linear_KP_", linear_KP);
        this->get_parameter<float>("linear_KI_", linear_KI);
        this->get_parameter<float>("linear_KD_", linear_KD);
        this->get_parameter<float>("angular_KP_", angular_KP);
        this->get_parameter<float>("angular_KI_", angular_KI);
        this->get_parameter<float>("angular_KD_", angular_KD);
        this->get_parameter<float>("targetDist_", targetDist);
        this->get_parameter<bool>("refresh_", refresh);

        // ================= 新增：自动初始化目标追踪框 =================
        int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
        this->get_parameter<int>("x1", x1);
        this->get_parameter<int>("y1", y1);
        this->get_parameter<int>("x2", x2);
        this->get_parameter<int>("y2", y2);

        // 核心逻辑：如果坐标不全为0，说明外部（大模型）传入了目标框
        if (x1 != 0 || y1 != 0 || x2 != 0 || y2 != 0) {
            // 将大模型的对角线坐标转换为 OpenCV 的 Rect (左上角坐标 x,y 和 宽高 width,height)
            selectRect.x = std::min(x1, x2);
            selectRect.y = std::min(y1, y2);
            selectRect.width = std::abs(x2 - x1);
            selectRect.height = std::abs(y2 - y1);
            
            // 重要：将标志位设为 true，假装用户已经松开了鼠标
            // 这会让 run_tracker.cpp 中的 imageCb 函数立刻执行 tracker.init()
            bRenewROI = true; 
            
            // 打印绿色的日志，提示参数接收成功
            RCLCPP_INFO(this->get_logger(), "自动接收到大模型目标框: 左上角x:%d, y:%d, 宽度:%d, 高度:%d", 
                        selectRect.x, selectRect.y, selectRect.width, selectRect.height);
        } else {
            RCLCPP_INFO(this->get_logger(), "未检测到外部坐标输入，等待手动框选...");
        }
        // ==============================================================

        // --- 4. 初始化底层的 PID 控制器实例 ---
        this->linear_PID = new PID(linear_KP, linear_KI, linear_KD); // 前进后退控制器
        this->angular_PID = new PID(angular_KP, angular_KI, angular_KD); // 左右转向控制器
        
        // 1. 声明专为传感器高频数据设计的 QoS (底层会自动匹配 BEST_EFFORT)
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
        
        // 2. 订阅彩色图像 (保持原有话题，加上 QoS)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", sensor_qos, std::bind(&ImageConverter::imageCb, this, _1));
            
        // 3. 订阅深度图像 (★关键★：换成真正有数据的对齐深度话题，并加上 QoS)
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/aligned_depth_to_color/image_raw", sensor_qos, std::bind(&ImageConverter::depthCb, this, _1));
        
        // 创建发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/KCF_image", 1);
        // 将输出重定向到中间话题，交给 Python 后台进行避障拦截过滤
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/kcf_cmd_vel", 1);
    }

    // 类成员变量声明
    PID *linear_PID;  // 线性速度 PID 指针
    PID *angular_PID; // 角速度 PID 指针

    // 窗口名称定义（无屏小车上如果设置了 offscreen 环境变量，这两个窗口不会实际弹出）
    const char *RGB_WINDOW = "rgb_img";
    const char *DEPTH_WINDOW = "depth_img";
    
    float targetDist = 1.0;       // 期望保持的距离
    float linear_speed = 0;       // 算出的前进速度
    float rotation_speed = 0;     // 算出的转向速度
    bool enable_get_depth = false;// 标志位：是否允许在深度图里测距（框选成功后才会开启）
    float dist_val[5];            // 存储目标框中心及周围共5个点的深度值
    
    // KCF 追踪器的配置开关
    bool HOG = true;              // 启用 HOG 特征（对形状敏感）
    bool FIXEDWINDOW = false;     // 窗口大小是否固定
    bool MULTISCALE = true;       // 启用多尺度追踪（目标变大变小自适应）
    bool LAB = false;             // 是否启用 LAB 颜色空间特征
    
    int center_x;                 // 目标框在画面中的中心 X 坐标
    KCFTracker tracker;           // KCF 追踪器核心实例对象
    
    // 成员函数声明
    void PIDcallback();

    void Reset();

    void Cancel();

    void imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg);

    void depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg);
};

#endif //TRANSBOT_ASTRA_KCF_TRACKER_H