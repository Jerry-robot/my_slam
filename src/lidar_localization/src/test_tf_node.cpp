#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/tf_listener/tf_listener.hpp"

using namespace lidar_localization;

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;
    ros::init(argc, argv, "test_tf_node");
    ros::NodeHandle nh;
    std::shared_ptr<TFListener> tf_world_to_imu = std::make_shared<TFListener>(nh, "world", "imu");
    std::shared_ptr<TFListener> tf_imu_to_lidar = std::make_shared<TFListener>(nh, "imu", "lidar");

    Eigen::Matrix4f world_to_imu = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f imu_to_lidar = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f world_to_lidar = Eigen::Matrix4f::Identity();

    ros::Rate rate(100);
    while (ros::ok()) {
        tf_world_to_imu->LookupData(world_to_imu);
        tf_imu_to_lidar->LookupData(imu_to_lidar);

        Eigen::Vector3f eulerAngle1 = world_to_imu.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
        Eigen::Vector3f eulerAngle2 = imu_to_lidar.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
        LOG(INFO) << "******************************************" << std::endl;
        eulerAngle1 = eulerAngle1 * 180 / M_PI;
        eulerAngle2 = eulerAngle2 * 180 / M_PI;
        LOG(INFO) << "world to imu:" << std::endl;
        LOG(INFO) << "translation:" << world_to_imu(0, 3) << " " << world_to_imu(1, 3) << " " << world_to_imu(2, 3)
                  << std::endl;
        LOG(INFO) << "rotate:" << eulerAngle1[0] << " " << eulerAngle1[1] << " " << eulerAngle1[2] << std::endl;

        LOG(INFO) << "imu to lidar is:" << std::endl;
        LOG(INFO) << "translation:" << imu_to_lidar(0, 3) << " " << imu_to_lidar(1, 3) << " " << imu_to_lidar(2, 3)
                  << std::endl;
        LOG(INFO) << "rotate:" << eulerAngle2[0] << " " << eulerAngle2[1] << " " << eulerAngle2[2] << std::endl;

        world_to_lidar = world_to_imu * imu_to_lidar;
        Eigen::Vector3f eulerAngle3 = world_to_lidar.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

        LOG(INFO) << "world to lidar is:" << std::endl;
        LOG(INFO) << "translation:" << world_to_lidar(0, 3) << " " << world_to_lidar(1, 3) << " " << world_to_lidar(2, 3)
                  << std::endl;
        LOG(INFO) << "rotate:" << eulerAngle3[0] << " " << eulerAngle3[1] << " " << eulerAngle3[2] << std::endl;

        Eigen::Vector4f point_in_lidar({1,0,0,1});
        Eigen::Vector4f point_in_world = world_to_lidar * point_in_lidar;
        LOG(INFO) << "point_in_lidar:" << point_in_lidar[0] << " " << point_in_lidar[1] << " " << point_in_lidar[2] << std::endl;
        LOG(INFO) << "point_in_world:" << point_in_world[0] << " " << point_in_world[1] << " " << point_in_world[2] << std::endl;


        rate.sleep();
    }

    return 0;
}
