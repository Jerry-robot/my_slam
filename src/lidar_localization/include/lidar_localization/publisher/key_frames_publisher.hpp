/**
 * @file key_frames_publisher.hpp
 * @author your name (you@domain.com)
 * @brief  key frames 信息发布
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFramesPublisher {
  public:
    KeyFramesPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame>& key_frames);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif