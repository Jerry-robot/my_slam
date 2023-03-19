/**
 * @file imu_subscriber.hpp
 * @author Jerry (1374450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */
/**
 * @file imu_data.hpp
 * @author Jerry (1377450529@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <string>
#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
class IMUSubscriber {
   public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;

    void ParseData(std::deque<IMUData>& deque_imu_data);

   private:
    void msg_callback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr);

   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<IMUData> new_imu_data_;
};

}  // namespace lidar_localization

#endif