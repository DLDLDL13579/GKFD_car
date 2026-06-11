#include <iostream>
#include "run_tracker.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "kcftracker.h"
#include <opencv2/core/core.hpp>
// 注释掉 highgui 避免引入任何窗口依赖
// #include <opencv2/highgui/highgui.hpp> 
#include <opencv2/opencv.hpp>

// 全局变量
Rect selectRect;
Point origin;
Rect result;
bool select_flag = false;
bool bRenewROI = false;
bool bBeginKCF = false;
Mat rgbimage;
Mat depthimage;

// 彻底移除鼠标回调函数，由大模型传参替代
/* void onMouse(int event, int x, int y, int, void *) { ... } 
*/

void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    linear_speed = 0;
    rotation_speed = 0;
    enable_get_depth = false;
    this->linear_PID->reset();
    this->angular_PID->reset();
    vel_pub_->publish(geometry_msgs::msg::Twist());
}

void ImageConverter::Cancel() {
    this->Reset();
}

void ImageConverter::PIDcallback() {
    this->targetDist=1.0;
    this->linear_PID->Set_PID(3.0, 0.0, 1.0);
    this->angular_PID->Set_PID(0.5, 0.0, 2.0);
    this->linear_PID->reset();
    this->angular_PID->reset();
}

void ImageConverter::imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常");
        return;
    }
    
    cv_ptr->image.copyTo(rgbimage);
    
    // 触发 KCF 初始化
    if (bRenewROI) {
         if (selectRect.width <= 0 || selectRect.height <= 0)
         {
             bRenewROI = false;
             return;
         }
        RCLCPP_INFO(this->get_logger(), ">> 正在初始化 KCF 算法模型...");
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        enable_get_depth = false;
        bRenewROI = false;
        RCLCPP_INFO(this->get_logger(), ">> KCF 初始化成功，开始追踪！");
    }

    if (bBeginKCF) {
        result = tracker.update(rgbimage);
        rectangle(rgbimage, result, Scalar(0, 255, 255), 1, 8);
        circle(rgbimage, Point(result.x + result.width / 2, result.y + result.height / 2), 3, Scalar(0, 0, 255),-1);
        enable_get_depth = true;
    } else {
        rectangle(rgbimage, selectRect, Scalar(255, 0, 0), 2, 8, 0);
    }

    // 图像回传
    sensor_msgs::msg::Image kcf_imagemsg;
    std_msgs::msg::Header _header;
    _header.stamp = this->get_clock()->now();
    cv_bridge::CvImage _cv_bridge(_header, sensor_msgs::image_encodings::BGR8, rgbimage);
    _cv_bridge.toImageMsg(kcf_imagemsg);
    image_pub_->publish(kcf_imagemsg);
}

void ImageConverter::depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    this->get_parameter<float>("targetDist_", this->targetDist);
    // D455 默认跟随时，严禁目标进入 0.52m 以内的物理盲区，所以目标距离最小设为 0.8m
    if(this->targetDist < 0.8) this->targetDist = 0.8; 

    cv_bridge::CvImagePtr cv_ptr;
    try {
        // 动态接受相机格式，不写死 TYPE_32FC1
        cv_ptr = cv_bridge::toCvCopy(msg); 
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "深度图格式转换失败");
        return; 
    }

    if (enable_get_depth) {
        int center_x = (int)(result.x + result.width / 2);
        int center_y = (int)(result.y + result.height / 2);
        
        // 越界保护防崩溃
        if (center_x < 5 || center_y < 5 || center_x >= depthimage.cols - 5 || center_y >= depthimage.rows - 5) {
            geometry_msgs::msg::Twist twist;
            vel_pub_->publish(twist);
            return;
        }

        // 自动兼容相机的 16 位整型格式，转换为米 (m)
        auto get_depth = [&](int y, int x) -> float {
            if (depthimage.type() == CV_32FC1) {
                return depthimage.at<float>(y, x) / 1000.0; 
            } else if (depthimage.type() == CV_16UC1) {
                return depthimage.at<uint16_t>(y, x) / 1000.0;
            }
            return 0.0;
        };

        dist_val[0] = get_depth(center_y - 5, center_x - 5);
        dist_val[1] = get_depth(center_y - 5, center_x + 5);
        dist_val[2] = get_depth(center_y + 5, center_x + 5);
        dist_val[3] = get_depth(center_y + 5, center_x - 5);
        dist_val[4] = get_depth(center_y, center_x);

        float distance = 0;
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++) {
            // 【针对 D455 的关键过滤】：剔除小于 0.55m 的盲区错误噪点
            if (dist_val[i] > 0.55 && dist_val[i] < 6.0) {
                distance += dist_val[i];
            } else {
                num_depth_points--;
            }
        }

        if (num_depth_points != 0) {
            distance /= num_depth_points;
            if (abs(distance - this->targetDist) < 0.1) linear_speed = 0;
            else linear_speed = -linear_PID->compute(this->targetDist, distance);
        } else {
            linear_speed = 0; // 陷入盲区，强制停止前进，防止乱撞
        }

        
        rotation_speed = angular_PID->compute(depthimage.cols / 2.0 / 100.0, center_x / 100.0);
        if (abs(rotation_speed) < 0.1) rotation_speed = 0;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_speed;
        twist.angular.z = rotation_speed;

        // 【新增智能逻辑】：如果目标卡在画面极左或极右边缘（即将丢失），且深度失效时，强制小车原地大角度旋转寻找
        if (num_depth_points == 0) {
            twist.linear.x = 0; // 找不到距离，绝对不前进
            if (center_x < 50) { // 目标消失在左边
                twist.angular.z = 0.8; // 快速向左原地打转寻找
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "目标在左侧丢失，向左旋转寻找...");
            } else if (center_x > depthimage.cols - 50) { // 目标消失在右边
                twist.angular.z = -0.8; // 快速向右原地打转寻找
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "目标在右侧丢失，向右旋转寻找...");
            }
        }

        // 【修改限幅】：放宽角速度限制，让小车转弯跟得上人 (由 0.35 放宽到 1.0)
        if (twist.linear.x > 0.35) twist.linear.x = 0.35;
        if (twist.linear.x < -0.35) twist.linear.x = -0.35;
        if (twist.angular.z > 1.0) twist.angular.z = 1.0;   // 放宽左转极限
        if (twist.angular.z < -1.0) twist.angular.z = -1.0; // 放宽右转极限
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "D455测距: %.2fm, V=%.2f, W=%.2f", distance, twist.linear.x, twist.angular.z);

        vel_pub_->publish(twist);
        // ========= 替换到这里结束 =========
    }
}

int main(int argc,char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConverter>();
    RCLCPP_INFO(node->get_logger(), "wheeltec_robot kcf_tracker node is spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}