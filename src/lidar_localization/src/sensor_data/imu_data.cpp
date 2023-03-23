/**
 * @file imu_data.cpp
 * @author Jerry
 * @brief
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/sensor_data/imu_data.hpp"
#include <cmath>
namespace lidar_localization {
/**
 * @brief 将IMU四元素数据转为矩阵形式
 *
 * @return Eigen::Matrix3f
 */
Eigen::Matrix3f IMUData::GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();
    return matrix;
}

/**
 * @brief 将IMU 数据插值统一到某一时间下
 *
 * @param unsynced_imu
 * @param sync_imu
 * @param sync_time
 * @return true
 * @return false
 */
bool IMUData::SyncData(std::deque<IMUData>& unsynced_imu, std::deque<IMUData>& sync_imu, double sync_time) {
    while (unsynced_imu.size() >= 2) {
        // 1、判断第一个数据时间是否小于sync_time
        if (unsynced_imu.front().time > sync_time)
            return false;
        // 2、判断第一个数据时间是否小于sync_time
        if (unsynced_imu.at(1).time < sync_time) {
            unsynced_imu.pop_front();
            continue;
        }
        // 判断第一个数据与sync_time时间差是否过大
        if (sync_time - unsynced_imu.front().time > 0.2) {
            unsynced_imu.pop_front();
            return false;
        }
        // 判断第二个数据与sync_time时间差是否过大
        if (unsynced_imu.at(1).time - sync_time > 0.2) {
            unsynced_imu.pop_front();
            return false;
        }
        break;
    }
    if (unsynced_imu.size() < 2)
        return false;

    IMUData front_data = unsynced_imu.at(0);
    IMUData back_data = unsynced_imu.at(1);

    IMUData sync_data;
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);

    sync_data.time = sync_time;
    sync_data.linear_acceleration.x =
        front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    sync_data.linear_acceleration.y =
        front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    sync_data.linear_acceleration.z =
        front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;

    sync_data.angular_velocity.x =
        front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    sync_data.angular_velocity.y =
        front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    sync_data.angular_velocity.z =
        front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    sync_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    sync_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    sync_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    sync_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;

    sync_data.orientation.Normlize();

    sync_imu.push_back(sync_data);
    return true;

}

}  // namespace lidar_localization