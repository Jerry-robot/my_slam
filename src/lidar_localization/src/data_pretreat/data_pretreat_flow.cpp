/**
 * @file data_pretreat_flow.cpp
 * @author Jerry
 * @brief 数据预处理 流程管理类
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 10000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 100000);
    imu_to_lidar_sub_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    gnss_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/velo_link", 100);

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;
    // LOG(INFO) << "Finish ReadData";
    if (!InitCalibration()) {
        return false;
    }
    if (!InitGNSS()) {
        LOG(INFO) << "初始化 GNSS";
        return false;
    }

    while (HasData()) {
        // LOG(INFO) << "读取数据";
        if (!IsValidData())
            continue;
        // LOG(INFO) << "成功更新 GNSS Odometry";
        TransformData();
        PublishData();
    }

    return true;
}

/**
 * @brief 读取传感器数据
 *
 * @return true
 * @return false
 */
bool DataPretreatFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSData> unsynced_gnss_;
    static std::deque<VelocityData> unsynced_velocity_;

    imu_sub_ptr_->ParseData(unsynced_imu_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);

    if (cloud_data_buff_.size() == 0)
        return false;

    double cloud_time = cloud_data_buff_.front().time;

    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_gnss || !valid_velocity) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

/**
 * @brief 判断是否已经获取 imu to lidar 的坐标变化
 *
 * @return true
 * @return false
 */

bool DataPretreatFlow::InitCalibration() {
    static bool has_init_transform = false;
    if (!has_init_transform) {
        if (imu_to_lidar_sub_ptr_->LookupData(imu_to_lidar_)) {
            LOG(INFO) << "初始化 imu to lidar 成功！";
            has_init_transform = true;
        }
        LOG(INFO) << "初始化 imu to lidar 失败！";
    }
    return has_init_transform;
}

/**
 * @brief 初始化GPS, 用第一帧当做初始位置
 *
 * @return true
 * @return false
 */
bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (gnss_data_buff_.size() > 0 && !gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }
    return gnss_inited;
}

/**
 * @brief 判断当前读取的点云缓存队列是否有数据
 *
 * @return true
 * @return false
 */
bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    // LOG(INFO) << "有数据";
    return true;
}

/**
 * @brief 去除时间相差过大的数据
 *
 * @return true
 * @return false
 */
bool DataPretreatFlow::IsValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    double diff_vel_time = current_cloud_data_.time - current_velocity_data_.time;

    if (diff_imu_time < -0.05 || diff_gnss_time < -0.05 || diff_vel_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    if (diff_vel_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    cloud_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    return true;
}

/**
 * @brief   将点云转换至lidar坐标系下
 *
 * @return true
 * @return false
 */
bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0, 3) = current_gnss_data_.local_E;
    gnss_pose_(1, 3) = current_gnss_data_.local_N;
    gnss_pose_(2, 3) = current_gnss_data_.local_U;
    gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= imu_to_lidar_;

    current_velocity_data_.TransformCoordinate(imu_to_lidar_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
    return true;
}

/**
 * @brief 发布 消息数据(gps里程计、雷达里程计)
 *
 * @return true
 * @return false
 */
bool DataPretreatFlow::PublishData() {
    LOG(INFO) << "处理==>数据预处理完毕,发布GNSS pose与点云";
    gnss_odom_pub_ptr_->Publish(gnss_pose_, current_cloud_data_.time);
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    return true;
}

}  // namespace lidar_localization