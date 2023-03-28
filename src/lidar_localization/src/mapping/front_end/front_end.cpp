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

#include "lidar_localization/mapping/front_end/front_end.hpp"
#include <pcl/common/transforms.h>
#include "lidar_localization/global_defination/global_defination.h.in"

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

namespace lidar_localization {
FrontEnd::FrontEnd() : local_map_ptr_(new CloudData::CLOUD) {
    // 设置默认参数

    InitWithConfig();
}
/**
 * @brief 获取config.yaml文件路径，用于初始化滤波与匹配模块
 *
 * @return true
 * @return false
 */
bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    InitParam(config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("local_map", local_filter_ptr_, config_node);
    InitRegistration(registration_ptr_, config_node);
    return true;
}
/**
 * @brief 初始化前端里程计参数
 *
 * @param config_node
 * @return true
 * @return false
 */
bool FrontEnd::InitParam(const YAML::Node& config_node) {
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    return true;
}

/**
 * @brief 初始化滤波类，并相关配置参数
 *
 * @param filter_user
 * @param filter_ptr
 * @param config_node
 * @return true
 * @return false
 */
bool FrontEnd::InitFilter(const std::string filter_user,
                          std::shared_ptr<CloudFilterInterface>& filter_ptr,
                          const YAML::Node& config_node) {
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << " 选择的滤波方法为: " << filter_method;
    if (filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    } else {
        LOG(ERROR) << "没有为 " << filter_user << "找到与" << filter_method << "相对应的滤波方法!";
        return false;
    }
    return true;
}

/**
 * @brief 初始化点云匹配类，并相关配置参数
 *
 * @param registration_user
 * @param registration_ptr
 * @param config_node
 * @return true
 * @return false
 */
bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,
                                const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO) << "选择的点云匹配方法为：" << registration_method;
    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没有找到与" << registration_method << " 相对应的点云匹配方式！";
        return false;
    }
    return true;
}

/**
 * @brief 返回雷达的位姿信息
 *
 * @param cloud_data
 * @return Eigen::Matrix4f
 */
bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);
    // 降采样的当前帧点云
    CloudData::CLOUD_PTR filter_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());

    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filter_cloud_ptr);
    // LOG(INFO) << "当前帧滤波!";

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图没有关键帧，代表这是第一帧
    // 用当前帧作为第一个关键帧，并更新局部地图容器和全局地图
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }
    registration_ptr_->ScanMatch(filter_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    LOG(INFO) << "前端==>点云匹配完成!";

    // 更新相邻帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 根据距离判断是否生成新的关键帧
    if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
        key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }
    return true;
}

/**
 * @brief 添加关键帧，输入ndt的目标点云，更新局部点云地图
 *
 * @param new_key_frame
 */
bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 进行值拷贝，获取new_key_frame的点云赋值给key_frame
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));

    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (static_cast<int>(local_map_frames_.size()) > local_frame_num_) {
        local_map_frames_.pop_front();
    }

    local_map_ptr_.reset(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    // 更新ndt匹配的滑窗区域的目标点云
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filter_local_map_ptr(new CloudData::CLOUD());
        local_filter_ptr_->Filter(local_map_ptr_, filter_local_map_ptr);
        registration_ptr_->SetInputTarget(filter_local_map_ptr);
    }

    return true;
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

}  // namespace lidar_localization