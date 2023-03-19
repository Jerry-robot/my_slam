/**
 * @file cloud_subscriber.hpp
 * @author Jerry (1377450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <deque>
#include <string>
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <glog/logging.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lidar_localization {
class CloudSubcriber {
   public:
    CloudSubcriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubcriber() = default;
    void ParseData(std::deque<CloudData>& cloud_data_buff);

   private:
    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);
   private:
    ros::NodeHandle nh_;
    ros::Subscriber subcriber;
    std::deque<CloudData> new_cloud_data_;
};


}  // namespace lidar_localization