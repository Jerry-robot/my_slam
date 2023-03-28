/**
 * @file odometry_publisher.hpp
 * @author Jerry (1374450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Dense>

namespace lidar_localization {
class OdometryPublisher {
   public:
    OdometryPublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix);
    void Publish(const Eigen::Matrix4f& transform_matrix, double data_time);
    void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time data_time);

   private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
};

}  // namespace lidar_localization

#endif