/**
 * @file imu_data.hpp
 * @author Jerry (1374450529@dqq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATIOIN_SENSOR_DATA_IMU_DATA_HPP_
#define LIDAR_LOCALIZATIOIN_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class IMUData {
   public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    struct Orientation {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;

   public:
    Eigen::Matrix3f GetOrientationMatrix() {
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f matrix = q.matrix().cast<float>();
        return matrix;
    }
};

}  // namespace lidar_localization

#endif
