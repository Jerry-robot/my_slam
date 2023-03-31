/**
 * @file loop_pose.cpp
 * @author your name (you@domain.com)
 * @brief   关键帧之间的相对位姿，用于回环检测
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 */
#include "lidar_localization/sensor_data/loop_pose.hpp"

namespace lidar_localization {
Eigen::Quaternionf LoopPose::GetOrientation() {
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);
    return q;
}
}  // namespace lidar_localization