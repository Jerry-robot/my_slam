/**
 * @file cloud_filter_interface.hpp
 * @author Jerrt
 * @brief   点云滤波模块
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_INTERFACE_HPP
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_INTERFACE_HPP

#include <yaml-cpp/yaml.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class CloudFilterInterface {
   public:
    virtual ~CloudFilterInterface() = default;
    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filter_cloud_ptr) = 0;
};

}  // namespace lidar_localization

#endif