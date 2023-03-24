/**
 * @file velocity_subscriber.cpp
 * @author Jerry
 * @brief ROS 速度订阅类
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/subscriber/velocity_subscriber.hpp"

namespace lidar_localization {
VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::MsgCallback, this);
}

void VelocitySubscriber::MsgCallback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr) {
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();

    velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;
    new_data_buff_.push_back(velocity_data);
}

void VelocitySubscriber::ParseData(std::deque<VelocityData>& twist_data_buff){
    if(new_data_buff_.size()>0){
        twist_data_buff.insert(twist_data_buff.end(), new_data_buff_.begin(), new_data_buff_.end());
        new_data_buff_.clear();
    }
}

}  // namespace lidar_localization