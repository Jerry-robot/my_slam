/**
 * @file imu_subscriber.cpp
 * @author Jerry (1374450529@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "lidar_localization/subscriber/imu_subscriber.hpp"

namespace lidar_localization {
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();

    imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    imu_data.orientation.w = imu_msg_ptr->orientation.w;
    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;

    new_imu_data_.push_back(imu_data);
    // std::cout<<"receive new imu data."<<std::endl;

}

void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {
    if (new_imu_data_.size() > 0) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
        std::cout<<"imu_data_buff size:"<<imu_data_buff.size()<<std::endl;
    }
}

}  // namespace lidar_localization
