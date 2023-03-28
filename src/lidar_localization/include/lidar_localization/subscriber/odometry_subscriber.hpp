/**
 * @file odometry_subscriber.hpp
 * @author Jerry
 * @brief   里程计订阅者
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <deque>
#include <string>
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
class OdometrySubscriber {
   public:
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    void ParseData(std::deque<PoseData>& pose_data_buff, std::string user_for = "");

   private:
    void MsgCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr);

   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PoseData> new_pose_data_;
};

}  // namespace lidar_localization

#endif