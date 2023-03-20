/**
 * @file test_frame_node.cpp
 * @author Jerry (1374450529@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h.in"

#include <lidar_localization/tf_listener/tf_listener.hpp>
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"

using namespace lidar_localization;


int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
        std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "imu_link", "velo_link");
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "/current_scan", 100, "/map");
    std::shared_ptr<OdometryPublisher> odometry_pub_ptr =
        std::make_shared<OdometryPublisher>(nh, "lidar_odom", "/map", "/lidar", 100);

    std::deque<IMUData> imu_data_buff;
    std::deque<CloudData> cloud_data_buff;
    std::deque<GNSSData> gnss_data_buff;
    // lidar_to_imu 其实是imu to lidar
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();

    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        // 读取数据
        imu_sub_ptr->ParseData(imu_data_buff);
        cloud_sub_ptr->ParseData(cloud_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received) {
            // base:"velo_link", child: "imu_link"
            if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
                transform_received = true;
                LOG(INFO)<<"lidar to imu transform matrix is:"<<std::endl<<lidar_to_imu;
            }
        } else {
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                CloudData cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();

                double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    if(!gnss_origin_position_inited)
                    {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }

                    Eigen::Matrix4f odometry_matrix;
                    // 雷达与IMU在一个坐标系下
                    gnss_data.UpdateXYZ();
                    odometry_matrix(0, 3) = gnss_data.local_E;
                    odometry_matrix(1, 3) = gnss_data.local_N;
                    odometry_matrix(2, 3) = gnss_data.local_U;
                    // 当前odometry_matrixwi 为imu_Link在世界坐标系中位姿
                    odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();

                    // Eigen::Vector3f eulerAngle1=odometry_matrix.block<3,3>(0,0).eulerAngles(0,1,2);
                    Eigen::Vector3f eulerAngle2=lidar_to_imu.block<3,3>(0,0).eulerAngles(0,1,2);
                    // LOG(INFO)<<"******************************************"<<std::endl;
                    // LOG(INFO)<<"map to imu transform matrix is:"<<std::endl<<odometry_matrix;
                    // eulerAngle1 = eulerAngle1*180/M_PI; 
                    eulerAngle2 = eulerAngle2*180/M_PI; 
                    // LOG(INFO)<<std::endl<<eulerAngle1[0]<<" "<<eulerAngle1[1]<<" "<<eulerAngle1[2]<<std::endl;
                    // LOG(INFO)<<"lidar to imu lidar_to_imu matrix is:"<<std::endl<<lidar_to_imu;
                    LOG(INFO)<<std::endl<<eulerAngle2[0]<<" "<<eulerAngle2[1]<<" "<<eulerAngle2[2]<<std::endl;


                    odometry_matrix *= lidar_to_imu;
                    
                    pcl::transformPointCloud(*(cloud_data.cloud_ptr), *(cloud_data.cloud_ptr), odometry_matrix);

                    odometry_pub_ptr->Publish(odometry_matrix);
                    cloud_pub_ptr->Publish(cloud_data.cloud_ptr);

                }
            }
        }

        rate.sleep();
    }







    return 0;
}
