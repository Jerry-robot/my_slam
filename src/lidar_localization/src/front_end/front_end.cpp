/**
 * @file front_end.cpp
 * @author Jerry
 * @brief 前段里程计
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/front_end/front_end.hpp"
#include <glog/logging.h>
namespace lidar_localization {
FrontEnd::FrontEnd()
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()),
      local_map_ptr_(new CloudData::CLOUD),
      global_map_ptr_(new CloudData::CLOUD),
      result_cloud_ptr_(new CloudData::CLOUD) {
    // 设置默认参数
    cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
    local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
    display_filter_.setLeafSize(0.5, 0.5, 0.5);

    ndt_ptr_->setResolution(1.0);
    ndt_ptr_->setStepSize(0.1);
    ndt_ptr_->setTransformationEpsilon(0.01);
    ndt_ptr_->setMaximumIterations(30);
}

/**
 * @brief 返回雷达的位姿信息
 *
 * @param cloud_data
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);
    // 降采样的当前帧点云
    CloudData::CLOUD_PTR filter_cloud_ptr(new CloudData::CLOUD());
    cloud_filter_.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    cloud_filter_.filter(*filter_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图没有关键帧，代表这是第一帧
    // 用当前帧作为第一个关键帧，并更新局部地图容器和全局地图
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateNewFrame(current_frame_);
        return current_frame_.pose;
    }
    // 设置ndt需要配准的点云
    ndt_ptr_->setInputSource(filter_cloud_ptr);
    ndt_ptr_->align(*result_cloud_ptr_, predict_pose);
    current_frame_.pose = ndt_ptr_->getFinalTransformation();

    // 更新相邻帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 根据距离判断是否生成新的关键帧
    if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
        2) {
        UpdateNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }
    return current_frame_.pose;
}

/**
 * @brief 添加关键帧，输入ndt的目标点云，更新全局与局部点云地图
 *
 * @param new_key_frame
 */
void FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 进行值拷贝，获取new_key_frame的点云赋值给key_frame
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));

    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > 20) {
        local_map_frames_.pop_front();
    }

    local_map_ptr_.reset(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // 更新ndt匹配的滑窗区域的目标点云
    if (local_map_frames_.size() < 10) {
        ndt_ptr_->setInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filter_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_.setInputCloud(local_map_ptr_);
        local_map_filter_.filter(*filter_local_map_ptr);
        ndt_ptr_->setInputTarget(filter_local_map_ptr);
    }

    // 更新全局地图
    global_map_frames_.push_back(key_frame);
    // 每添加一百帧点云更新全局点云
    // LOG(INFO) << "global_map_frames_size: " << global_map_frames_.size() << std::endl;

    if (global_map_frames_.size() % 5 != 0) {
        return;
    } else {

        global_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < global_map_frames_.size(); ++i) {
            pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr,
                                     global_map_frames_.at(i).pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }
        has_new_global_map_ = true;
    }
}

/**
 * @brief 设置初始位姿
 *
 * @param init_pose
 * @return true
 * @return false
 */
bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

/**
 * @brief Set the Predict Pose object
 *
 * @param predict_pose
 * @return true
 * @return false
 */
bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose) {
    predict_pose_ = predict_pose;
    return true;
}
/**
 * @brief Get the New Local Map object
 *
 * @param local_map_ptr
 * @return true
 * @return false
 */
bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if (has_new_local_map_) {
        display_filter_.setInputCloud(local_map_ptr_);
        display_filter_.filter(*local_map_ptr);
        return true;
    }
    return false;
}

/**
 * @brief Get the New Global Map object
 *
 * @param global_map_ptr
 * @return true
 * @return false
 */
bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if (has_new_global_map_) {
        display_filter_.setInputCloud(global_map_ptr_);
        display_filter_.filter(*global_map_ptr);
        return true;
    }
    return false;
}

/**
 * @brief Get the Current Scan object
 *
 * @param current_scan_ptr
 * @return true
 * @return false
 */
bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    display_filter_.setInputCloud(result_cloud_ptr_);
    display_filter_.filter(*current_scan_ptr);
    return true;
}

}  // namespace lidar_localization