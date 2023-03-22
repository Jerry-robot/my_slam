/**
 * @file front_end_flow_node.cpp
 * @author Jerry
 * @brief 前端里程计
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <glog/logging.h>
#include <ros/ros.h>

#include <lidar_localization/saveMap.h>
#include <boost/filesystem.hpp>
#include "lidar_localization/front_end/front_end_flow.hpp"
#include "lidar_localization/global_defination/global_defination.h.in"
using namespace lidar_localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(saveMap::Request& request, saveMap::Response& response) {
    response.succeed = _front_end_flow_ptr->SaveMap();
    _front_end_flow_ptr->PublishGlobalMap();
    return response.succeed;
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_flow_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        _front_end_flow_ptr->Run();

        rate.sleep();
    }
    std::string key_frame_path = "/home/gjw/my_slam/src/lidar_localization/slam_data/key_frames";
    if (boost::filesystem::is_directory(key_frame_path)) {
        boost::filesystem::remove_all(key_frame_path);
        LOG(INFO) << "删除所有关键帧!";
    }
    return 0;
}