/**
 * @file pose_data.hpp
 * @author Jerry
 * @brief   位姿数据
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class PoseData {
   public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

   public:
    Eigen::Quaternionf GetOrientation();
};

}  // namespace lidar_localization

#endif