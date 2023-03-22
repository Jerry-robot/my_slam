/**
 * @file registration_interface.hpp
 * @author Jerry
 * @brief   点云配准模块的基类
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_REGISTRATION_INTERFACE_HPP_

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace lidar_localization {
class RegistrationInterface {
   public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                           const Eigen::Matrix4f& predict_pose,
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) = 0;
};

}  // namespace lidar_localization

#endif