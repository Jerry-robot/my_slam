/**
 * @file data_pretreat_node.cpp
 * @author Jerry
 * @brief  数据预处理节点
 * @version 0.1
 * @date 2023-03-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"
#include "lidar_localization/global_defination/global_defination.h.in"
#include <glog/logging.h>

using namespace lidar_localization;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    std::shared_ptr<DataPretreatFlow> data_pretreat_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);
    LOG(INFO)<<"”处理==>启动数据预处理模块！";
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        data_pretreat_ptr->Run();        
        rate.sleep();
    }
    return 0;
}
