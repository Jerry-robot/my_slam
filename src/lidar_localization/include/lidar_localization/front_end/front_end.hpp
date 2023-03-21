/**
 * @file front_end.hpp
 * @author Jerry
 * @brief 前段里程计
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class FrontEnd {
   public:
    class Frame {
       public:
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

   public:
    FrontEnd();

    Eigen::Matrix4f Update(const CloudData& cloud_data);

    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

   private:
    void UpdateNewFrame(const Frame& new_key_frame);

   private:
    // 单帧滤波
    pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
    // 局部地图降采样滤波
    pcl::VoxelGrid<CloudData::POINT> local_map_filter_;
    // 全局地图降采样滤波
    pcl::VoxelGrid<CloudData::POINT> display_filter_;

    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    // 局部点云指针
    CloudData::CLOUD_PTR local_map_ptr_;
    // 全局点云指针
    CloudData::CLOUD_PTR global_map_ptr_;
    // ndt对齐后的点云指针
    CloudData::CLOUD_PTR result_cloud_ptr_;
    // 去除异常值的当前帧点云与位姿
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
};

}  // namespace lidar_localization

#endif