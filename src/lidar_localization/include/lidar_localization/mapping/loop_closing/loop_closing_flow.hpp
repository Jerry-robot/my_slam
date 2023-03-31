/**
 * @file loop_closing_flow.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_FLOW_HPP_

#include <ros/ros.h>
#include <deque>
#include "lidar_localization/subscriber/key_frame_subscriber.hpp"
#include "lidar_localization/publisher/loop_pose_publisher.hpp"
#include "lidar_localization/mapping/loop_closing/loop_closing.hpp"

namespace lidar_localization {
class LoopClosingFlow {
   public:
    LoopClosingFlow(ros::NodeHandle& nh);

    bool Run();

   private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

   private:
    // subscriber
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;

    // publisher
    std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;

    // loop closing
    std::shared_ptr<LoopClosing> loop_closing_ptr_;

    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> key_gnss_buff_;

    KeyFrame current_key_frame_;
    KeyFrame current_key_gnss_;

};

}  // namespace lidar_localization

#endif