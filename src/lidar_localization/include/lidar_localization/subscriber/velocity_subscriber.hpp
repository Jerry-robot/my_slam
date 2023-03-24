/**
 * @file velocity_subscriber.hpp
 * @author Jerry
 * @brief
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <string>
#include <deque>
#include <geometry_msgs/TwistStamped.h>
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class VelocitySubscriber {
   public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& twist_data_buff);

   private:
    void MsgCallback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_data_buff_;
};

}  // namespace lidar_localization

#endif