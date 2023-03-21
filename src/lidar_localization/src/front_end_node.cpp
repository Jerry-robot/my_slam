/**
 * @file front_end_node.cpp
 * @author Jerry
 * @brief
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>

#include <deque>

#include <Eigen/Dense>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"

#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"

#include "lidar_localization/front_end/front_end.hpp"

using namespace lidar_localization;

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
        std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 10000);
    std::shared_ptr<TFListener> imu_to_lidar_sub_ptr = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "/current_scan", 1000, "/map");
    std::shared_ptr<CloudPublisher> local_cloud_pub_ptr =
        std::make_shared<CloudPublisher>(nh, "local_cloud", 1000, "map");
    std::shared_ptr<CloudPublisher> global_cloud_pub_ptr =
        std::make_shared<CloudPublisher>(nh, "global_cloud", 1000, "map");
    std::shared_ptr<OdometryPublisher> lidar_odom_pub_ptr =
        std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    std::shared_ptr<OdometryPublisher> gnss_odom_pub_ptr =
        std::make_shared<OdometryPublisher>(nh, "gnss_odom", "map", "lidar", 100);

    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;
    std::deque<CloudData> cloud_data_buff;

    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR local_map_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR current_scan_ptr(new CloudData::CLOUD());

    std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>();
    // map to imu的坐标变化是否获取
    bool transform_received = false;
    // gps 是否初始化
    bool gnss_origin_position_inited = false;
    // 前段里程计是否初始化
    bool front_end_inited = false;

    double run_time = 0.0;
    double init_time = 0.0;
    bool time_inited = false;
    bool has_global_map_published = false;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        // 更新数据
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);
        cloud_sub_ptr->ParseData(cloud_data_buff);

        // imu to lidar
        Eigen::Matrix4f imu_to_ldar = Eigen::Matrix4f::Identity();
        if (!transform_received) {
            if (imu_to_lidar_sub_ptr->LookupData(imu_to_ldar)) {
                transform_received = true;
            }
        } else {
            while (imu_data_buff.size() > 0 && gnss_data_buff.size() > 0 && cloud_data_buff.size() > 0) {
                // 处理数据
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();
                CloudData cloud_data = cloud_data_buff.front();
                if (!time_inited) {
                    init_time = cloud_data.time;
                    time_inited = true;
                }
                run_time = cloud_data.time - init_time;
                // LOG(INFO) << "RUN TIME: " << run_time << std::endl;
                // 对齐数据的时间
                double d_time = imu_data.time - cloud_data.time;
                if (d_time > 0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time < -0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }

                    gnss_data.UpdateXYZ();
                    Eigen::Matrix4f odomery_matrix = Eigen::Matrix4f::Identity();
                    odomery_matrix(0, 3) = gnss_data.local_E;
                    odomery_matrix(1, 3) = gnss_data.local_N;
                    odomery_matrix(2, 3) = gnss_data.local_U;
                    odomery_matrix.block<3, 3>(0, 0) = imu_data.GetOrientationMatrix();

                    odomery_matrix *= imu_to_ldar;
                    gnss_odom_pub_ptr->Publish(odomery_matrix);

                    if (!front_end_inited) {
                        front_end_ptr->SetInitPose(odomery_matrix);
                        front_end_inited = true;
                    }
                    front_end_ptr->SetPredictPose(odomery_matrix);

                    Eigen::Matrix4f laser_matrix = front_end_ptr->Update(cloud_data);
                    lidar_odom_pub_ptr->Publish(laser_matrix);

                    front_end_ptr->GetCurrentScan(current_scan_ptr);
                    cloud_pub_ptr->Publish(current_scan_ptr);

                    if (front_end_ptr->GetNewLocalMap(local_map_ptr)) {
                        local_cloud_pub_ptr->Publish(local_map_ptr);
                    }
                }

                if (run_time > 450 && !has_global_map_published) {
                    // LOG(INFO) << "PUBLISH" << std::endl;

                    if (front_end_ptr->GetNewGlobalMap(global_map_ptr)) {
                        global_cloud_pub_ptr->Publish(global_map_ptr);
                        LOG(INFO) << "PUBLISH GLOABL MAP: " << global_map_ptr->points.size() << std::endl;
                        // has_global_map_published = true;
                    }
                }
            }
        }
        rate.sleep();
    }

    return 0;
}
