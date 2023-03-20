/**
 * @file tf_listener.cpp
 * @author Jerry (1374450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/tf_listener/tf_listener.hpp"
#include <Eigen/Geometry>
#include <glog/logging.h>
namespace lidar_localization {
TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {}

/**
 * @brief 获取base_frame_id  to child_frame_id_ 的坐标轴变化
 *          用来将child_frame_id_中的点转移到获取base_frame_id坐标系下
 * @param transform_matrix
 * @return true
 * @return false
 */
bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try {
        tf::StampedTransform transform;
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
        TransformToMatrix(transform, transform_matrix);
        return true;
    } catch (tf::TransformException& ex) {
        return false;
    }
}

/**
 * @brief StampedTransform ---> matrix
 *
 * @param 输入：transform
 * @param 输出：transform_matrix
 * @return true
 * @return false
 */
bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(),
                                 transform.getOrigin().getZ());

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    // LOG(INFO)<<"roll: "<<roll/M_PI*180<<" pitch: "<<pitch/M_PI*180<<" yaw: "<<yaw/M_PI*180<<std::endl;
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    return true;
}

}  // namespace lidar_localization
