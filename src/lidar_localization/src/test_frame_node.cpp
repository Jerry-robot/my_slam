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
#include "glog/logging.h"

#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/global_defination/global_defination.h.in"

using namespace lidar_localization;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/oxts/pointcloud", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);

    

    std::deque<IMUData> imu_data_buff;
    std::deque<CloudData> cloud_data_buff;
    std::deque<GNSSData> gnss_data_buff;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        // 读取数据
        imu_sub_ptr->ParseData(imu_data_buff);
        cloud_sub_ptr->ParseData(cloud_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);
        

        rate.sleep();
    }
    

    return 0;
}
