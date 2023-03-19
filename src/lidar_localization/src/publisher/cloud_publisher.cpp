/**
 * @file cloud_publisher.cpp
 * @author Jerry (1377450529@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "lidar_localization/publisher/cloud_publisher.hpp"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh, std::string topic_name, size_t buff_size, std::string frame_id)
    : nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}
void CloudPublisher::Publish(CloudData::CLOUD_PTR cloud_ptr_input) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    cloud_ptr_output->header.frame_id = frame_id_;
    cloud_ptr_output->header.stamp = ros::Time::now();
    publisher_.publish(*cloud_ptr_output);
}

}  // namespace lidar_localization