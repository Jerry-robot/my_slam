/**
 * @file loop_pose.hpp
 * @author your name (you@domain.com)
 * @brief   关键帧之间的相对位姿，用于回环检测
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP
#define LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP

#include <Eigen/Dense>

namespace lidar_localization {
class LoopPose {
   public:
    double time = 0.0;
    unsigned int index0 = 0;
    unsigned int index1 = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

   public:
    Eigen::Quaternionf GetOrientation();
};

}  // namespace lidar_localization

#endif