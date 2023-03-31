/**
 * @file loop_pose_publisher.hpp
 * @author your name (you@domain.com)
 * @brief   发布闭环检测结果
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lidar_localization/sensor_data/loop_pose.hpp"

namespace lidar_localization {
class LoopPosePublisher {
   public:
    LoopPosePublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);

    LoopPosePublisher() = default;

    void Publish(LoopPose& loop_pose);

    bool HasSubscribers();

   private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};

}  // namespace lidar_localization