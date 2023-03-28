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

using namespace lidar_localization;

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;
    std::shared_ptr<BackEndFlow> bcak_end_flow_ptr = std::make_shared<BackEndFlow>(nh);
    LOG(INFO) << "后端==>启动后端优化模块！";
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        bcak_end_flow_ptr->Run();
        rate.sleep();
    }
    std::string key_frame_path = "/home/gjw/my_slam/src/lidar_localization/slam_data/key_frames";
    if (boost::filesystem::is_directory(key_frame_path)) {
        boost::filesystem::remove_all(key_frame_path);
        LOG(INFO) << "后端==> 删除所有关键帧!";
    }
    return 0;
}
