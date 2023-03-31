/**
 * @file loop_closing_node.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <glog/logging.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/mapping/loop_closing/loop_closing_flow.hpp"
using namespace lidar_localization;

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "loop_closing_node");
    ros::NodeHandle nh;
    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);
    LOG(INFO) << "回环==>启动回环模块！";
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_closing_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}