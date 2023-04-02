/**
 * @file front_end_node.cpp
 * @author 前端里程计模块节点
 * @brief
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <glog/logging.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/mapping/viewer/viewer_flow.hpp"
#include "lidar_localization/saveMap.h"

using namespace lidar_localization;

std::shared_ptr<ViewerFlow> _front_end_flow_ptr;

bool _need_save_map = false;

bool save_map_callback(saveMap::Request& request, saveMap::Response& response){
    _need_save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_flow_node");
    ros::NodeHandle nh;
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    _front_end_flow_ptr = std::make_shared<ViewerFlow>(nh, cloud_topic);
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    LOG(INFO) << "可视==>启动可视化模块！";
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        _front_end_flow_ptr->Run();
        if(_need_save_map){
            _front_end_flow_ptr->SaveMap();
            _need_save_map = false;
        }
        rate.sleep();
    }
    return 0;
}