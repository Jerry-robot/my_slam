/**
 * @file loop_pose_subscriber.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/subscriber/loop_pose_subscriber.hpp"

namespace lidar_localization {
LoopPoseSubscriber::LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size):nh_(nh){
    subscriber_ = nh_.subscribe(topic_name, buff_size, &LoopPoseSubscriber::MsgCallback, this);
}

void LoopPoseSubscriber::ParseData(std::deque<LoopPose>& loop_data_buff){
    if (new_loop_data_.size() > 0) {
        loop_data_buff.insert(loop_data_buff.end(), new_loop_data_.begin(), new_loop_data_.end());
        new_loop_data_.clear();
    }
}

void LoopPoseSubscriber::MsgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_data_ptr){
    LoopPose loop_pose;
    loop_pose.time = loop_data_ptr->header.stamp.toSec();

    loop_pose.pose(0, 3) = loop_data_ptr->pose.pose.position.x;
    loop_pose.pose(1, 3) = loop_data_ptr->pose.pose.position.y;
    loop_pose.pose(2, 3) = loop_data_ptr->pose.pose.position.z;

    Eigen::Quaternionf q(loop_data_ptr->pose.pose.orientation.w, loop_data_ptr->pose.pose.orientation.x,
                         loop_data_ptr->pose.pose.orientation.y, loop_data_ptr->pose.pose.orientation.z);
    loop_pose.pose.block<3, 3>(0, 0) = q.matrix();

    loop_pose.index0 = (unsigned int)loop_data_ptr->pose.covariance.at(0);
    loop_pose.index1 = (unsigned int)loop_data_ptr->pose.covariance.at(1);


    new_loop_data_.push_back(loop_pose);
}
}  // namespace lidar_localization