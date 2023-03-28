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

#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_

#include <Eigen/Dense>
#include <deque>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/registration/ndt_registration.hpp"
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
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

   private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitFilter(const std::string filter_user,
                    std::shared_ptr<CloudFilterInterface>& filter_ptr,
                    const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);

   private:

    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_filter_ptr_;

    std::shared_ptr<RegistrationInterface> registration_ptr_;

    std::deque<Frame> local_map_frames_;

    // 局部点云指针
    CloudData::CLOUD_PTR local_map_ptr_;
    
    // 去除异常值的当前帧点云与位姿
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;
};

}  // namespace lidar_localization

#endif