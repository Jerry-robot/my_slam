/**
 * @file velocity_data.hpp
 * @author Jerry
 * @brief 速度数据
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <Eigen/Dense>
#include <deque>

namespace lidar_localization {
class VelocityData {
   public:
    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

   public:
    static bool SyncData(std::deque<VelocityData>& unsync_data,
                         std::deque<VelocityData>& synced_data,
                         double sync_time);
    void TransfromCoordinate(const Eigen::Matrix4f& transform_matrix);
};

}  // namespace lidar_localization

#endif