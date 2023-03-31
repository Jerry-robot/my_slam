/**
 * @file view_flow.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "lidar_localization/mapping/viewer/viewer.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/viewer/viewer_flow.hpp"

namespace lidar_localization {
/**
 * @brief Construct a new Viewer Flow:: Viewer Flow object
 *          读取输入：同步时间点云、世界坐标系雷达里程计、关键帧、优化后的关键帧
 *          输出发布：优化后里程计、当前帧点云、局部地图、全局地图
 * @param nh
 */
ViewerFlow::ViewerFlow(ros::NodeHandle& nh) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
    transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 100000);
    optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 100000);
    // publisher
    optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odom", "/map", "/lidar", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // viewer
    viewer_ptr_ = std::make_shared<Viewer>();
}

bool ViewerFlow::Run() {
    if (!ReadData())
        return false;

    while (HasData()) {
        if (ValidData()) {
            viewer_ptr_->UpdateWithNewKeyFrame(key_frame_buff_, current_transformed_odom_, current_cloud_data_);
            PublishLocalData();
        }
    }
    if (optimized_key_frames_.size() > 0) {
        viewer_ptr_->UpdateWithOptimizedKeyFrames(optimized_key_frames_);
        PublishGlobalData();
    }

    return true;
}

/**
 * @brief 读取传感器数据
 *
 * @return true
 * @return false
 */
bool ViewerFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
    key_frame_sub_ptr_->ParseData(key_frame_buff_);
    optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_);

    return true;
}

/**
 * @brief 判断当前读取的点云缓存队列是否有数据
 *
 * @return true
 * @return false
 */
bool ViewerFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (transformed_odom_buff_.size() == 0)
        return false;

    return true;
}

/**
 * @brief 去除时间相差过大的数据
 *
 * @return true
 * @return false
 */
bool ViewerFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_transformed_odom_ = transformed_odom_buff_.front();

    double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;

    if (diff_odom_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_odom_time > 0.05) {
        transformed_odom_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    transformed_odom_buff_.pop_front();

    return true;
}


/**
 * @brief 发布 消息数据(gps里程计、雷达里程计)
 *
 * @return true
 * @return false
 */
bool ViewerFlow::PublishData() {
    optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
    current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());

    if (viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        viewer_ptr_->GetLocalMap(cloud_ptr);
        local_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

bool ViewerFlow::PublishLocalData() {
    optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
    current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());
    if (viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        viewer_ptr_->GetLocalMap(cloud_ptr);
        local_map_pub_ptr_->Publish(cloud_ptr);
    }
    return true;
}

bool ViewerFlow::PublishGlobalData() {
    if (viewer_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        viewer_ptr_->GetGlobalMap(cloud_ptr);
        global_map_pub_ptr_->Publish(cloud_ptr);
    }
    return true;
}

bool ViewerFlow::SaveMap() {
    return viewer_ptr_->SaveMap();
}
}  // namespace lidar_localization