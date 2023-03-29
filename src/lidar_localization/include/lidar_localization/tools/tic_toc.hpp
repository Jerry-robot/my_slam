/**
 * @file tic_toc.hpp
 * @author your name (you@domain.com)
 * @brief   记录程序运行时间
 * @version 0.1
 * @date 2023-03-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace lidar_localization {
class TicToc {
   public:
    TicToc(){
        tic();
    }
    void tic(){
        start = std::chrono::system_clock::now();
    }
    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end -start;
        start = std::chrono::system_clock::now();
        return elapsed_seconds.count();
    }
   private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

}  // namespace lidar_localization