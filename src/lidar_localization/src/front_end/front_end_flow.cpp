/**
 * @file front_end_flow.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/front_end/front_end_flow.hpp"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 10000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 100000);
    imu_to_lidar_sub_ptr_ = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", 1000, "/map");
    local_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_cloud", 1000, "map");
    global_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_cloud", 1000, "map");

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

    lidar_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    gnss_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss_odom", "map", "lidar", 100);

    front_end_ptr_ = std::make_shared<FrontEnd>();

    global_map_ptr_.reset(new CloudData::CLOUD());
    local_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
    // LOG(INFO) << "成功构造 FrontEndFlow！";
}

bool FrontEndFlow::Run() {
    // LOG(INFO) << "开始运行程序！";
    ReadData();
    if (!InitCalibration()) {
        // LOG(INFO) << "标定IMU";
        return false;
    }
    if (!InitGNSS()) {
        // LOG(INFO) << "初始化 GNSS";
        return false;
    }

    while (HasData()) {
        if (!IsValidData())
            continue;
        // LOG(INFO) << "更新 GNSS Odometry";
        UpdateGNSSOdometry();
        // LOG(INFO) << "成功更新 GNSS Odometry";
        if (UpdateLaserOdometry()) {
            // LOG(INFO) << "更新 Laser Odometry";
            PublishData();
            SaveTrajectory();
        }
    }

    return true;
}

/**
 * @brief 读取传感器数据
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::ReadData() {
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
bool FrontEndFlow::InitCalibration() {
    static bool has_init_transform = false;
    if (!has_init_transform) {
        if (imu_to_lidar_sub_ptr_->LookupData(imu_to_lidar_)) {
            has_init_transform = true;
        }
    }
    return has_init_transform;
}

/**
 * @brief 初始化GPS, 用第一帧当做初始位置
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::InitGNSS() {
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
bool FrontEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    return true;
}

/**
 * @brief 去除时间相差过大的数据
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::IsValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();

    double d_time = current_cloud_data_.time - current_imu_data_.time;
    if (d_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }
    if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
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
 * @brief 更新里程计数据，获取map to lidar 的位姿变化作为里程计数据
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::UpdateGNSSOdometry() {
    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0, 3) = current_gnss_data_.local_E;
    gnss_odometry_(1, 3) = current_gnss_data_.local_N;
    gnss_odometry_(2, 3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= imu_to_lidar_;

    return true;
}

/**
 * @brief   更新前端里程计
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::UpdateLaserOdometry() {
    // 将速度旋转至雷达坐标系
    current_velocity_data_.TransfromCoordinate(imu_to_lidar_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    static bool front_end_pose_inited = false;
    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
        laser_odometry_ = gnss_odometry_;

        return true;
    }
    laser_odometry_ = Eigen::Matrix4f::Identity();
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

/**
 * @brief 发布 消息数据(gps里程计、雷达里程计、当前帧点云、局部点云)
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::PublishData() {
    gnss_odom_pub_ptr_->Publish(gnss_odometry_);
    lidar_odom_pub_ptr_->Publish(laser_odometry_);

    if (front_end_ptr_->GetCurrentScan(current_scan_ptr_))
        cloud_pub_ptr_->Publish(current_scan_ptr_);
    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        local_cloud_pub_ptr_->Publish(local_map_ptr_);

    return true;
}
/**
 * @brief 保存全局地图到文件中
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::SaveMap() {
    return front_end_ptr_->SaveMap();
}

/**
 * @brief 发布全局地图
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::PublishGlobalMap() {
    if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) {
        global_cloud_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CLOUD());
        return true;
    }
    return false;
}

/**
 * @brief 保存里程计数据到文件中
 *
 * @return true
 * @return false
 */
bool FrontEndFlow::SaveTrajectory() {
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;

    if (!is_file_created) {
        std::string data_path = WORK_SPACE_PATH + "/slam_data/trajectory";
        if (!FileManager::CreateDirtectory(data_path))
            return false;
        std::string file_gnss_path = data_path + "/gnss_odometry.txt";
        if (!FileManager::CreateFile(ground_truth, file_gnss_path)) {
            return false;
        }
        LOG(INFO) << "GNSS 里程计保存地址为: " << file_gnss_path;

        std::string file_laser_path = data_path + "/laser_odometry.txt";
        if (!FileManager::CreateFile(laser_odom, file_laser_path)) {
            return false;
        }
        LOG(INFO) << "Laser 里程计保存地址为: " << file_laser_path;
        is_file_created = true;
    }

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
                ground_truth << " ";
                laser_odom << " ";
            }
        }
    }
    return true;
}

}  // namespace lidar_localization