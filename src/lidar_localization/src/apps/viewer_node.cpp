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
using namespace lidar_localization;

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_flow_node");
    ros::NodeHandle nh;
    std::shared_ptr<ViewerFlow> _front_end_flow_ptr = std::make_shared<ViewerFlow>(nh);
    LOG(INFO)<<"可视==>启动可视化模块！";
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        _front_end_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}