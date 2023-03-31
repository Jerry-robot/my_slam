/**
 * @file loop_closing_flow.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/mapping/loop_closing/loop_closing_flow.hpp"
#include <glog/logging.h>

namespace lidar_localization {

LoopClosingFlow::LoopClosingFlow(ros::NodeHandle& nh) {
    // subscriber
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
    key_gnss_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_gnss", 100000);

    // publisher
    loop_pose_pub_ptr_ = std::make_shared<LoopPosePublisher>(nh, "/loop_pose", "/map", 100);

    // loop closing
    loop_closing_ptr_ = std::make_shared<LoopClosing>();
    // LOG(INFO) << "LoopClosingFlow finish!";
}

bool LoopClosingFlow::Run() {
    if (!ReadData())
        return false;
    // LOG(INFO) << "ReadData finish!";

    while (HasData()) {
        // LOG(INFO) << "HasData finish!";

        if (!ValidData())
            return false;
        // LOG(INFO) << "ValidData finish!";
        loop_closing_ptr_->Update(current_key_frame_, current_key_gnss_);
        // LOG(INFO) << "Update FINISH!";

        if (PublishData())
            LOG(INFO) << "PublishData finish!";
    }
}

bool LoopClosingFlow::ReadData() {
    key_frame_sub_ptr_->ParseData(key_frame_buff_);
    key_gnss_sub_ptr_->ParseData(key_gnss_buff_);
    return true;
}

bool LoopClosingFlow::HasData() {
    if (key_frame_buff_.size() == 0)
        return false;
    if (key_gnss_buff_.size() == 0)
        return false;
    return true;
}

bool LoopClosingFlow::ValidData() {
    current_key_frame_ = key_frame_buff_.front();
    current_key_gnss_ = key_gnss_buff_.front();

    double diff_gnss_time = current_key_frame_.time - current_key_gnss_.time;

    if (diff_gnss_time < -0.05) {
        key_frame_buff_.pop_front();
        return false;
    }
    if (diff_gnss_time > 0.05) {
        key_gnss_buff_.pop_front();
        return false;
    }

    key_frame_buff_.pop_front();
    key_gnss_buff_.pop_front();
    return true;
}

bool LoopClosingFlow::PublishData() {
    if (loop_closing_ptr_->HasNewLoopPose()) {
        loop_pose_pub_ptr_->Publish(loop_closing_ptr_->GetCurrentLoopPose());
        return true;
    }
    return false;
}

}  // namespace lidar_localization