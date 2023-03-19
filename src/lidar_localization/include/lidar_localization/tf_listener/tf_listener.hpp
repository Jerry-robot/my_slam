/**
 * @file tf_listener.hpp
 * @author Jerry (1374450529@qq.com)
 * @brief   tf 坐标系监听模块
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_TF_LISTENER_TF_LISTENER_HPP
#define LIDAR_LOCALIZATION_TF_LISTENER_TF_LISTENER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <string>

namespace lidar_localization {
class TFListener {
   public:
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;

    bool LookupData(Eigen::Matrix4f& transform_matrix);

   private:
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

   private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};

}  // namespace lidar_localization

#endif