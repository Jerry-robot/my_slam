/**
 * @file data_pretreat_flow.hpp
 * @author Jerry
 * @brief   数据预处理 流程管理类
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP

#include <ros/ros.h>
#include <Eigen/Dense>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"

#include "lidar_localization/tf_listener/tf_listener.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

namespace lidar_localization {
class DataPretreatFlow {
   public:
    DataPretreatFlow(ros::NodeHandle& nh);
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);
    bool Run();

   private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool IsValidData();
    bool UpdateGNSSOdometry();
    bool TransformData();
    bool PublishData();


   private:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<TFListener> imu_to_lidar_sub_ptr_;

    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_odom_pub_ptr_;

    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    std::deque<CloudData> cloud_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;

    IMUData current_imu_data_;
    GNSSData current_gnss_data_;
    CloudData current_cloud_data_;
    VelocityData current_velocity_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f imu_to_lidar_ = Eigen::Matrix4f::Identity();
};

}  // namespace lidar_localization

#endif