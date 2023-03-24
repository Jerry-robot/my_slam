/**
 * @file distortion_adjust.hpp
 * @author  Jerry
 * @brief 匀速模型 激光雷达点云去畸变
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class DistortionAdjust {
   public:
    void SetMotionInfo(float scan_period, VelocityData current_velocity);
    bool AdjustCloud(CloudData::CLOUD_PTR& origin_cloud_ptr, CloudData::CLOUD_PTR& target_cloud_ptr);

   private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;

   private:
    Eigen::Matrix3f UpdateMatrix(float real_time);
};



}  // namespace lidar_localization