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
CloudPublisher::CloudPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id,size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}
void CloudPublisher::Publish(CloudData::CLOUD_PTR cloud_ptr_input) {
    PublishData(cloud_ptr_input, ros::Time::now());
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR cloud_ptr_input, double data_time) {
    PublishData(cloud_ptr_input, ros::Time(data_time));
}


void CloudPublisher::PublishData(CloudData::CLOUD_PTR cloud_ptr_input, ros::Time ros_time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    cloud_ptr_output->header.frame_id = frame_id_;
    cloud_ptr_output->header.stamp = ros_time;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}

}  // namespace lidar_localization