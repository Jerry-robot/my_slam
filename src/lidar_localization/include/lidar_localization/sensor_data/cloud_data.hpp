/**
 * @file cloud_data.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization {
class CloudData {
   public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

   public:
    CloudData() : cloud_ptr(new CLOUD()) {}

   public:
    CLOUD_PTR cloud_ptr;
    double time = 0.0;
};

}  // namespace lidar_localization

#endif