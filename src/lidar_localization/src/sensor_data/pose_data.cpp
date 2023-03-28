/**
 * @file pose_data.cpp
 * @author  Jerry
 * @brief   位姿数据类
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
Eigen::Quaternionf PoseData::GetOrientation() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
}
}  // namespace lidar_localization