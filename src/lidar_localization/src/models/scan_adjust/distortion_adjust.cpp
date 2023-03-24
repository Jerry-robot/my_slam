/**
 * @file distortion_adjust.cpp
 * @author Jerry
 * @brief 匀速模型去畸变
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include <pcl/common/transforms.h>
#include <cmath>

namespace lidar_localization {
/**
 * @brief 设置运动模型参数
 *
 * @param scan_period
 * @param current_velocity
 */
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData current_velocity) {
    scan_period_ = scan_period;
    velocity_[0] = current_velocity.linear_velocity.x;
    velocity_[1] = current_velocity.linear_velocity.y;
    velocity_[2] = current_velocity.linear_velocity.z;

    angular_rate_ << current_velocity.angular_velocity.x, current_velocity.angular_velocity.y,
        current_velocity.angular_velocity.z;
}

/**
 * @brief 根据运动模型调整点云位姿
 *
 * @param input_cloud_ptr
 * @param output_cloud_ptr
 */
bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) {
    CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
    output_cloud_ptr->points.clear();

    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();

    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
    // 旋转到雷达坐标系点云旋转至0方向坐标系
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    for (size_t point_index = 0; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].y);
        if (orientation < 0) {
            orientation += orientation_space;
        }
        if (orientation < delete_space || orientation_space - orientation < delete_space)
            continue;
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;
        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x, origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);
        Eigen::Matrix3f current_matrix = this->UpdateMatrix(real_time);
        Eigen::Vector3f rotate_point = current_matrix * origin_point;
        Eigen::Vector3f adjust_point = rotate_point + velocity_ * real_time;
        CloudData::POINT point;
        point.x = adjust_point[0];
        point.y = adjust_point[1];
        point.z = adjust_point[2];

        output_cloud_ptr->push_back(point);
    }
    // 转移至雷达坐标系
    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());

    return true;
}

/**
 * @brief 获取当前时刻的旋转量
 *
 * @param real_time
 * @return Eigen::Matrix3f
 */
Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle[2], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle[1], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle[0], Eigen::Vector3f::UnitX());

    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}


}  // namespace lidar_localization