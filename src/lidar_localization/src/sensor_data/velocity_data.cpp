/**
 * @file velocity_data.cpp
 * @author Jerry
 * @brief
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
/**
 * @brief  将速度数据插值生成某一时刻下的速度
 *
 * @param unsync_data
 * @param synced_data
 * @param sync_time
 * @return true
 * @return false
 */
bool VelocityData::SyncData(std::deque<VelocityData>& unsync_data,
                            std::deque<VelocityData>& synced_data,
                            double sync_time) {
    while (unsync_data.size() >= 2) {
        // 1、判断第一个数据时间是否小于sync_time
        if (unsync_data.front().time > sync_time)
            return false;
        // 2、判断第一个数据时间是否小于sync_time
        if (unsync_data.at(1).time < sync_time) {
            unsync_data.pop_front();
            continue;
        }
        // 判断第一个数据与sync_time时间差是否过大
        if (sync_time - unsync_data.front().time > 0.2) {
            unsync_data.pop_front();
            return false;
        }
        // 判断第二个数据与sync_time时间差是否过大
        if (unsync_data.at(1).time - sync_time > 0.2) {
            unsync_data.pop_front();
            return false;
        }
        break;
    }
    if (unsync_data.size() < 2)
        return false;

    VelocityData front_data = unsync_data.at(0);
    VelocityData back_data = unsync_data.at(1);
    VelocityData sync_data;

    sync_data.time = sync_time;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - back_data.time) / (back_data.time - front_data.time);

    sync_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    sync_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    sync_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;

    sync_data.angular_velocity.x =
        front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    sync_data.angular_velocity.y =
        front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    sync_data.angular_velocity.z =
        front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    synced_data.push_back(sync_data);
    return true;
}

/**
 * @brief 将速度进行坐标旋转
 * 
 * @param transform_matrix 
 */
void VelocityData::TransformCoordinate(const Eigen::Matrix4f& transform_matrix) {
    Eigen::Vector3f linear_velocity_temp(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    Eigen::Vector3f angular_velocity_temp(angular_velocity.x, angular_velocity.y, angular_velocity.z);

    Eigen::Matrix3f rotate_matrix = transform_matrix.block<3, 3>(0, 0);

    linear_velocity_temp = rotate_matrix * linear_velocity_temp;
    angular_velocity_temp = rotate_matrix * angular_velocity_temp;

    Eigen::Vector3f r(transform_matrix(0, 3), transform_matrix(1, 3), transform_matrix(2, 3));
    Eigen::Vector3f delta_v;
    // 由旋转产生的速度量 delta_v = w * r
    delta_v(0) = angular_velocity_temp(1) * r(2) - angular_velocity_temp(2) * r(1);
    delta_v(1) = angular_velocity_temp(2) * r(0) - angular_velocity_temp(0) * r(2);
    // delta_v(2) = angular_velocity_temp(1) * r(1) - angular_velocity_temp(1) * r(0); //原始程序
    delta_v(2) = angular_velocity_temp(0) * r(1) - angular_velocity_temp(1) * r(0);

    linear_velocity_temp += delta_v;

    linear_velocity.x = linear_velocity_temp[0];
    linear_velocity.y = linear_velocity_temp[1];
    linear_velocity.z = linear_velocity_temp[2];

    angular_velocity.x = angular_velocity_temp[0];
    angular_velocity.y = angular_velocity_temp[1];
    angular_velocity.z = angular_velocity_temp[2];
}

}  // namespace lidar_localization