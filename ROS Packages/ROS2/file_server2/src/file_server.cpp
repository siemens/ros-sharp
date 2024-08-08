/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

- Added ROS2 support.
- Added mutex and lock guards to make the save_file_callback thread-safe.
    (C) Siemens AG, 2024, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "file_server2/srv/get_binary_file.hpp"
#include "file_server2/srv/save_binary_file.hpp"
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <mutex>
#include <thread>
#include <string>

using namespace std::placeholders;

class FileServerNode : public rclcpp::Node
{
public:
    FileServerNode() : Node("file_server")
    {
        get_file_service_ = this->create_service<file_server2::srv::GetBinaryFile>(
            "/file_server/get_file",
            std::bind(&FileServerNode::get_file_callback, this, _1, _2));

        save_file_service_ = this->create_service<file_server2::srv::SaveBinaryFile>(
            "/file_server/save_file",
            std::bind(&FileServerNode::save_file_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "ROS2 File Server initialized.");
    }

private:
    void get_file_callback(
        const std::shared_ptr<file_server2::srv::GetBinaryFile::Request> request,
        std::shared_ptr<file_server2::srv::GetBinaryFile::Response> response)
    {
        if (request->name.compare(0, 10, "package://") != 0)
        {
            RCLCPP_INFO(this->get_logger(), "Only \"package://\" addresses allowed.");
            return;
        }

        std::string address = request->name.substr(10);
        std::string package = address.substr(0, address.find("/"));
        std::string filepath = address.substr(package.length());
        std::string directory = ament_index_cpp::get_package_share_directory(package);
        directory += filepath;

        std::ifstream inputfile(directory.c_str(), std::ios::binary);
        if (!inputfile.is_open())
        {
            RCLCPP_INFO(this->get_logger(), "File \"%s\" not found.", request->name.c_str());
            return;
        }

        response->value.assign(
            std::istreambuf_iterator<char>(inputfile),
            std::istreambuf_iterator<char>());

        RCLCPP_INFO(this->get_logger(), "get_file request: %s", request->name.c_str());
    }

    void create_directories(const std::string& packagePath, const std::string& filePath)
    {
        if (filePath.empty())
        {
            return;
        }

        create_directories(packagePath, filePath.substr(0, filePath.find_last_of("/")));

        struct stat sb;
        if (!(stat((packagePath + filePath).c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)))
        {
            mkdir((packagePath + filePath).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }
    }

    std::string generate_ros_package(const std::string& packageName)
    {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory(packageName);
        std::string directory = package_share_dir;

        std::string packageXmlContents = "<?xml version=\"1.0\"?>\n<package format=\"2\">\n  <name>" + packageName + "</name>\n  <version>0.0.0</version>\n  <description>The " + packageName + " package</description>\n\n  <maintainer email=\"ros-sharp.ct@siemens.com\">Ros#</maintainer>\n\n  <license>Apache 2.0</license>\n\n  <buildtool_depend>ament_cmake</buildtool_depend>\n  <build_depend>rclcpp</build_depend>\n  <exec_depend>rclcpp</exec_depend>\n\n  <export>\n  </export>\n</package>";

        std::ofstream xmlfile(directory + "/package.xml", std::ios::out | std::ios::binary);
        xmlfile << packageXmlContents;
        xmlfile.close();

        std::string cmakelist_content = "cmake_minimum_required(VERSION 3.5)\nproject(" + packageName + ")\n\nfind_package(ament_cmake REQUIRED)\nfind_package(rclcpp REQUIRED)\n\nadd_executable(" + packageName + "_node src/" + packageName + "_node.cpp)\nament_target_dependencies(" + packageName + "_node rclcpp)\n\ninstall(TARGETS " + packageName + "_node\n  DESTINATION lib/" + packageName + ")\n\nament_package()";

        std::ofstream cmakefile(directory + "/CMakeLists.txt", std::ios::out | std::ios::binary);
        cmakefile << cmakelist_content;
        cmakefile.close();

        return directory;
    }

    void save_file_callback(
        const std::shared_ptr<file_server2::srv::SaveBinaryFile::Request> request,
        std::shared_ptr<file_server2::srv::SaveBinaryFile::Response> response)
    {
        std::lock_guard<std::mutex> guard(save_file_mutex_);

        RCLCPP_INFO(this->get_logger(), "save_file request: %s", request->name.c_str());

        if (request->name.compare(0, 10, "package://") != 0)
        {
            RCLCPP_INFO(this->get_logger(), "Only \"package://\" addresses allowed.");
            return;
        }

        std::string address = request->name.substr(10);
        std::string package = address.substr(0, address.find("/"));
        std::string filepath = address.substr(package.length());
        std::string directory;

        try {
            directory = ament_index_cpp::get_package_share_directory(package);
        } catch (const std::exception &e) {
            RCLCPP_INFO(this->get_logger(), "Package \"%s\" not found. Creating a new package.", package.c_str());
            directory = generate_ros_package(package);
        }

        create_directories(directory, filepath.substr(0, filepath.find_last_of("/")));
        directory += filepath;

        std::ofstream file_to_save(directory.c_str(), std::ios::binary);
        if (!file_to_save.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", directory.c_str());
            return;
        }

        file_to_save.write(reinterpret_cast<const char*>(request->value.data()), request->value.size());
        file_to_save.close();

        response->name = request->name;

        RCLCPP_INFO(this->get_logger(), "save_file request completed: %s", request->name.c_str());
    }

    rclcpp::Service<file_server2::srv::GetBinaryFile>::SharedPtr get_file_service_;
    rclcpp::Service<file_server2::srv::SaveBinaryFile>::SharedPtr save_file_service_;
    std::mutex save_file_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FileServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
