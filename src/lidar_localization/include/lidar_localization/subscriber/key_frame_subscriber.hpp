/**
 * @file key_frame_subscriber.hpp
 * @author your name (you@domain.com)
 * @brief   订阅 key frame 数据
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFrameSubscriber {
  public:
    KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    KeyFrameSubscriber() = default;
    void ParseData(std::deque<KeyFrame>& key_frame_buff);

  private:
    void msg_callback(const geometry_msgs::PoseStampedConstPtr& key_frame_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<KeyFrame> new_key_frame_; 
};
}
#endif