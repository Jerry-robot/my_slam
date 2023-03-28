/**
 * @file cloud_publisher.hpp
 * @author Jerry (1377450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <lidar_localization/sensor_data/cloud_data.hpp>

namespace lidar_localization {
class CloudPublisher {
   public:
    CloudPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, size_t buff_size);
    CloudPublisher() = default;
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input);
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input, double data_time);
    bool HasSubscribers();

   private:
    void PublishData(CloudData::CLOUD_PTR cloud_ptr_input, ros::Time ros_time);

   private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};

}  // namespace lidar_localization

#endif