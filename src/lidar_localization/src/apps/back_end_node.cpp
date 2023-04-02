/**
 * @file back_end_node.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <glog/logging.h>
#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"
#include "lidar_localization/optimizeMap.h"


using namespace lidar_localization;


std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request& request, optimizeMap::Response& response){
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}


int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;
    
    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    _back_end_flow_ptr = std::make_shared<BackEndFlow>(nh, cloud_topic, odom_topic);

    ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);

    LOG(INFO) << "后端==>启动后端优化模块！";
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        _back_end_flow_ptr->Run();
        if(_need_optimize_map){
            _back_end_flow_ptr->ForceOptimize();
            _need_optimize_map = false;
        }

        rate.sleep();
    }
    std::string key_frame_path = "/media/gjw/Elements/slam_data/slam_data/key_frames";
    if (boost::filesystem::is_directory(key_frame_path)) {
        boost::filesystem::remove_all(key_frame_path);
        LOG(INFO) << "后端==> 删除所有关键帧!";
    }
    return 0;
}
