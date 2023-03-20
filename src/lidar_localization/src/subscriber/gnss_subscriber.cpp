/**
 * @file gnss_subscriber.cpp
 * @author Jerry (1374450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/subscriber/gnss_subscriber.hpp"
namespace lidar_localization {
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::GNSSCallback, this);
}

/**
 * @brief 读取gps放入类的队列中
 */
void GNSSSubscriber::GNSSCallback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();

    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;

    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);
    // std::cout<<"receive new gnss data."<<std::endl;

}

/**
 * @brief 将类内存好的gps数据传出去
 * 
 * @param gnss_data_buff 引用传值
 */
void GNSSSubscriber::ParseData(std::deque<GNSSData> &gnss_data_buff) {
    if(new_gnss_data_.size())
    {
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
        // std::cout<<"gnss_data_buff size:"<<gnss_data_buff.size()<<std::endl;

    }
}

}  // namespace lidar_localization