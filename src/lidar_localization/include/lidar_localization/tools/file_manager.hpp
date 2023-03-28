/**
 * @file file_manager.hpp
 * @author Jerry
 * @brief 目录与文件创建类
 * @version 0.1
 * @date 2023-03-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <string>

namespace lidar_localization {
class FileManager {
   public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string dirtectory_path);
    static bool InitDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string dirtectory_path, std::string use_for);
};

}  // namespace lidar_localization

#endif