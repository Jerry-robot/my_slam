/**
 * @file cloud_subscriber.cpp
 * @author Jerry (1374450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/subscriber/cloud_subscriber.hpp"

namespace lidar_localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subcriber = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::CloudCallback, this);
}

void CloudSubscriber::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();

    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    new_cloud_data_.push_back(cloud_data);
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    if(new_cloud_data_.size()){
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}
}  // namespace lidar_localization