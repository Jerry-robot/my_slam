/**
 * @file odometry_subscriber.cpp
 * @author Jerry
 * @brief   里程计订阅者
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/subscriber/odometry_subscriber.hpp"
#include <glog/logging.h>
namespace lidar_localization {
OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::MsgCallback, this);
}
void OdometrySubscriber::ParseData(std::deque<PoseData>& pose_data_buff, std::string user_for) {
    if (new_pose_data_.size() > 0) {
        // LOG(INFO) << user_for << " new_pose_data_ size " << new_pose_data_.size() << std::endl;

        pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
        new_pose_data_.clear();
        // LOG(INFO) << user_for << " pose_data_buff size " << pose_data_buff.size() << std::endl;

    }
}

void OdometrySubscriber::MsgCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr) {
    PoseData pose_data;
    pose_data.time = odom_ptr->header.stamp.toSec();

    pose_data.pose(0, 3) = odom_ptr->pose.pose.position.x;
    pose_data.pose(1, 3) = odom_ptr->pose.pose.position.y;
    pose_data.pose(2, 3) = odom_ptr->pose.pose.position.z;

    Eigen::Quaternionf q(odom_ptr->pose.pose.orientation.w, odom_ptr->pose.pose.orientation.x,
                         odom_ptr->pose.pose.orientation.y, odom_ptr->pose.pose.orientation.z);
    pose_data.pose.block<3, 3>(0, 0) = q.matrix();
    new_pose_data_.push_back(pose_data);
    // LOG(INFO) << " -----------------------" << std::endl;
    // LOG(INFO) << " new_pose_data_ size " << new_pose_data_.size() << std::endl;
    // LOG(INFO) << " -----------------------" << std::endl;
}

}  // namespace lidar_localization