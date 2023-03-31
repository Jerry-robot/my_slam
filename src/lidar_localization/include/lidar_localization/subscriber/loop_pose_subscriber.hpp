/**
 * @file loop_pose_subscriber.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <deque>
#include "lidar_localization/sensor_data/loop_pose.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace lidar_localization {
class LoopPoseSubscriber {
   public:
    LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<LoopPose>& loop_data_buff);

   private:
    void MsgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_data_ptr);
   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<LoopPose> new_loop_data_;    

};


}  // namespace lidar_localization

#endif