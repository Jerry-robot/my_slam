/**
 * @file front_end_flow.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/mapping/front_end/front_end_flow.hpp"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1000);
    lidar_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_odom", "map", "lidar", 100);
    front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
    // LOG(INFO) << "开始运行程序！";
    if (!ReadData())
        return false;

    while (HasData()) {
        if (!IsValidData())
            continue;
        UpdateLaserOdometry();
        LOG(INFO) << "前端==>发布 Laser Odometry";
        PublishData();
    }

    return true;
}

/**
 * @brief 读取传感器数据
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

/**
 * @brief 判断当前读取的点云缓存队列是否有数据
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

/**
 * @brief 去除时间相差过大的数据
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::IsValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();
    return true;
}

/**
 * @brief   更新前端里程计
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::UpdateLaserOdometry() {
    // 将速度旋转至雷达坐标系
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        laser_odometry_ = Eigen::Matrix4f::Identity();
        return front_end_ptr_->SetInitPose(laser_odometry_);
    }
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

/**
 * @brief 发布 消息数据(gps里程计、雷达里程计、当前帧点云、局部点云)
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::PublishData() {
    lidar_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    return true;
}

}  // namespace lidar_localization