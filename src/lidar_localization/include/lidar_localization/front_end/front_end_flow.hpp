/**
 * @file front_end_flow.hpp
 * @author Jerry
 * @brief 前段里程计流程放在类中实现
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP

#include <ros/ros.h>

#include <Eigen/Dense>
#include <deque>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"

#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"

#include "lidar_localization/front_end/front_end.hpp"

#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {
class FrontEndFlow {
   public:
    FrontEndFlow(ros::NodeHandle& nh);
    bool Run();
    bool SaveMap();
    bool PublishGlobalMap();

   private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool IsValidData();
    bool UpdateGNSSOdometry();
    bool UpdateLaserOdometry();
    bool PublishData();
    bool SaveTrajectory();

   private:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<TFListener> imu_to_lidar_sub_ptr_;

    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_cloud_pub_ptr_;

    std::shared_ptr<OdometryPublisher> lidar_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_odom_pub_ptr_;

    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    std::deque<CloudData> cloud_data_buff_;

    IMUData current_imu_data_;
    GNSSData current_gnss_data_;
    CloudData current_cloud_data_;

    // std::shared_ptr<CloudData::CLOUD> global_map_ptr_;
    // std::shared_ptr<CloudData::CLOUD> local_map_ptr_;
    // std::shared_ptr<CloudData::CLOUD> current_scan_ptr_;

    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    std::shared_ptr<FrontEnd> front_end_ptr_;

    Eigen::Matrix4f gnss_odometry_;
    Eigen::Matrix4f laser_odometry_;
    Eigen::Matrix4f imu_to_lidar_;
};

}  // namespace lidar_localization

#endif