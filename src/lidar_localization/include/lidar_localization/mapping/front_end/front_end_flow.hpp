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

#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_EDN_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_EDN_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include <Eigen/Dense>
#include <deque>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

#include "lidar_localization/mapping/front_end/front_end.hpp"


namespace lidar_localization {
class FrontEndFlow {
   public:
    FrontEndFlow(ros::NodeHandle& nh);
    bool Run();

   private:
    bool ReadData();
    bool HasData();
    bool IsValidData();
    bool UpdateLaserOdometry();
    bool PublishData();

   private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;

    std::shared_ptr<OdometryPublisher> lidar_odom_pub_ptr_;

    std::deque<CloudData> cloud_data_buff_;

    CloudData current_cloud_data_;

    std::shared_ptr<FrontEnd> front_end_ptr_;
    Eigen::Matrix4f laser_odometry_;
};

}  // namespace lidar_localization

#endif