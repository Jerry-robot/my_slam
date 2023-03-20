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

namespace lidar_localization {
TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
        
    }

/**
 * @brief 获取child_frame_id_ to base_frame_id 的坐标变化
 * 
 * @param transform_matrix 
 * @return true 
 * @return false 
 */
bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix){
    try{
        tf::StampedTransform transform;
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
        TransformToMatrix(transform, transform_matrix);
        return true;
    }catch(tf::TransformException &ex){
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
bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix){
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    transform_matrix =(tl_btol * rot_z_btol*rot_y_btol*rot_x_btol).matrix();
    return true;
}

}  // namespace lidar_localization