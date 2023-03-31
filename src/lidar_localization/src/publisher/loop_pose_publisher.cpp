/**
 * @file loop_pose_publisher.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/publisher/loop_pose_publisher.hpp"
#include <glog/logging.h>
#include <Eigen/Dense>

namespace lidar_localization {
LoopPosePublisher::LoopPosePublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size)
    : nh_(nh), frame_id_(frame_id) {
        publisher_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
    }

void LoopPosePublisher::Publish(LoopPose& loop_pose) {
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;

    ros::Time ros_time((float)loop_pose.time);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.pose.pose.position.x = loop_pose.pose(0, 3);
    pose_stamped.pose.pose.position.y = loop_pose.pose(1, 3);
    pose_stamped.pose.pose.position.z = loop_pose.pose(2, 3);

    Eigen::Quaternionf q = loop_pose.GetOrientation();
    pose_stamped.pose.pose.orientation.x = q.x();
    pose_stamped.pose.pose.orientation.y = q.y();
    pose_stamped.pose.pose.orientation.z = q.z();
    pose_stamped.pose.pose.orientation.w = q.w();

    pose_stamped.pose.covariance[0] = (double)loop_pose.index0;
    pose_stamped.pose.covariance[1] = (double)loop_pose.index1;

    publisher_.publish(pose_stamped);
}

bool LoopPosePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers()!= 0;
}

}  // namespace lidar_localization