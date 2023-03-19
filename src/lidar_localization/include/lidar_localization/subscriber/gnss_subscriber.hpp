/**
 * @file gnss_subscriber.hpp
 * @author Jerry (1377450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <deque>
#include "lidar_localization/sensor_data/gnss_data.hpp"

namespace lidar_localization {
class GNSSSubscriber {
   public:
    GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber()=default;
    void ParseData(std::deque<GNSSData> &gnss_data_buff);

   private:
    void GNSSCallback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GNSSData> new_gnss_data_;
};

}  // namespace lidar_localization

#endif