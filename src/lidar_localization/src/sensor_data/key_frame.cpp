/**
 * @file key_frame.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
Eigen::Quaternionf KeyFrame::GetQuaternion(){
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
}

}