/**
 * @file key_frame.hpp
 * @author Jerry
 * @brief
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class KeyFrame {
   public:
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    unsigned int index = 0;

   public:
    Eigen::Quaternionf GetQuaternion();
};
}  // namespace lidar_localization

#endif