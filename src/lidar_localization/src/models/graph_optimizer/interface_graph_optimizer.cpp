/**
 * @file interface_graph_optimizer.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace lidar_localization {
void InterfaceGraphOptimizer::SetMaxIterationNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}  // namespace lidar_localization