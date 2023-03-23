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
        LOG(WARNING) << "GeoConverter has not set origin position.";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

/**
 * @brief 将GNSS数据插值统一到点云时间下
 *
 * @param unsync_gnss
 * @param synced_gnss
 * @param sync_time
 * @return true
 * @return false
 */
bool GNSSData::SyncData(std::deque<GNSSData>& unsync_gnss, std::deque<GNSSData>& synced_gnss, double sync_time) {
    while (unsync_gnss.size() >= 2) {
        // 1、判断第一个数据时间是否小于sync_time
        if (unsync_gnss.front().time > sync_time)
            return false;
        // 2、判断第一个数据时间是否小于sync_time
        if (unsync_gnss.at(1).time < sync_time) {
            unsync_gnss.pop_front();
            continue;
        }
        // 判断第一个数据与sync_time时间差是否过大
        if (sync_time - unsync_gnss.front().time > 0.2) {
            unsync_gnss.pop_front();
            return false;
        }
        // 判断第二个数据与sync_time时间差是否过大
        if (unsync_gnss.at(1).time - sync_time > 0.2) {
            unsync_gnss.pop_front();
            return false;
        }
        break;
    }

    if (unsync_gnss.size() < 2)
        return false;

    GNSSData front_data = unsync_gnss.at(0);
    GNSSData back_data = unsync_gnss.at(1);
    GNSSData sync_data;
    sync_data.time = sync_time;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);

    sync_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    sync_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    sync_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;

    sync_data.local_E = front_data.local_E * front_scale + back_data.local_E + back_scale;
    sync_data.local_N = front_data.local_N * front_scale + back_data.local_N + back_scale;
    sync_data.local_U = front_data.local_U * front_scale + back_data.local_U + back_scale;

    synced_gnss.push_back(sync_data);
    return true;
}

}  // namespace lidar_localization
