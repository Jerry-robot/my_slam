/**
 * @file gnss_data.cpp
 * @author Jerry (1374450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/sensor_data/gnss_data.hpp"
#include <glog/logging.h>

bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {
void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}
void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING)<<"GeoConverter has not set origin position.";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

}  // namespace lidar_localization
