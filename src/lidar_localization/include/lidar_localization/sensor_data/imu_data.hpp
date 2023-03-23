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
#include <cmath>
#include <deque>

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
       public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

       public:
        void Normlize() {
            double norm = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(w, 2));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;

   public:
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<IMUData>& unsynced_imu, std::deque<IMUData>& sync_imu, double cloud_time);
};

}  // namespace lidar_localization

#endif
