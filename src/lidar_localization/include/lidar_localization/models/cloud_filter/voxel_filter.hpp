/**
 * @file voxel_filter.hpp
 * @author Jerry
 * @brief   体素滤波实现
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>
namespace lidar_localization {
class VoxelFilter : public CloudFilterInterface {
   public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filter_cloud_ptr) override;

   private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

   private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};

}  // namespace lidar_localization

#endif