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
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <file_server/GetBinaryFile.h>
#include <file_server/SaveBinaryFile.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

bool get_file(
	file_server::GetBinaryFile::Request &req,
	file_server::GetBinaryFile::Response &res)
{

	// analyse request:
	if (req.name.compare(0,10,"package://")!=0)
	{
		ROS_INFO("only \"package://\" addresses allowed.");
		return true;
	}	
	std::string address = req.name.substr(10);
	std::string package = address.substr(0,address.find("/"));
	std::string filepath = address.substr(package.length());
	std::string directory = ros::package::getPath(package);
	directory+=filepath;
	
	// open file:
	std::ifstream inputfile(directory.c_str(),std::ios::binary);

	// stop if file does not exist:
	if(!inputfile.is_open())
	{
		ROS_INFO("file requested \"%s\" not found.\n", directory.c_str());
		return true;
	}

	// read file contents:	
	res.value.assign(
		(std::istreambuf_iterator<char>(inputfile)),
		std::istreambuf_iterator<char>());

	ROS_INFO("get_file request: %s\n", req.name.c_str());
	//ROS_INFO("package: %s\n",package.c_str());
	//ROS_INFO("filepath: %s\n",filepath.c_str());
	//ROS_INFO("directory: %s\n",directory.c_str());

	return true;
}

//A recursive method that goes through a path and creates all parent directories
void create_directories(std::string packagePath, std::string filePath) 
{
	if(filePath.size() == 0)
	{
		return;
	}
	
	create_directories(packagePath, filePath.substr(0, filePath.find_last_of("/")));
	
	struct stat sb;
	if (!(stat((packagePath + filePath).c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) //Check if path is already a directory
	{
		mkdir((packagePath + filePath).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	}
	
}
std::string generate_ros_package(std::string packageName)
{	
	std::string catkin_folder = std::getenv("ROS_PACKAGE_PATH");
	catkin_folder = catkin_folder.substr(0, catkin_folder.find(":"));
	
	
	std::string directory = catkin_folder + "/" + packageName;
	mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	//Create default package.xml file
	std::string packageXmlContents = "<?xml version=\"1.0\"?>\n<package format=\"2\">\n  <name>" + packageName + "</name>\n  <version>0.0.0</version>\n  <description>The " + packageName + " package</description>\n\n  <maintainer email=\"ros-sharp.ct@siemens.com\">Ros#</maintainer>\n\n  <license>Apache 2.0</license>\n\n  <buildtool_depend>catkin</buildtool_depend>\n  <build_depend>roscpp</build_depend>\n  <build_depend>rospy</build_depend>\n  <build_depend>std_msgs</build_depend>\n  <build_export_depend>roscpp</build_export_depend>\n  <build_export_depend>rospy</build_export_depend>\n  <build_export_depend>std_msgs</build_export_depend>\n  <exec_depend>roscpp</exec_depend>\n  <exec_depend>rospy</exec_depend>\n  <exec_depend>std_msgs</exec_depend>\n\n  <export>\n  </export>\n</package>";

	std::ofstream xmlfile;
	xmlfile.open ((directory + "/package.xml").c_str(), std::ios::out | std::ios::ate | std::ios::binary);
	xmlfile << packageXmlContents;
	xmlfile.close();
	
	//Create default CMakeLists.txt file
	std::string cmakelist_content = "cmake_minimum_required(VERSION 2.8.3)\nproject(" + packageName + ")\n\nfind_package(catkin REQUIRED COMPONENTS\n  roscpp\n  rospy\n  std_msgs\n)\n\ninclude_directories(\n  ${catkin_INCLUDE_DIRS}\n)";

	std::ofstream cmakefile;
	cmakefile.open ((directory + "/CMakeLists.txt").c_str(), std::ios::out | std::ios::ate | std::ios::binary);
	cmakefile << cmakelist_content;
	cmakefile.close();

	return directory;
}

bool save_file(
	file_server::SaveBinaryFile::Request &req,
	file_server::SaveBinaryFile::Response &res)
{
	if (req.name.compare(0,10,"package://")!=0)
	{
		ROS_INFO("only \"package://\" addresses allowed.");
		return true;
	}
	std::string address = req.name.substr(10);
	std::string package = address.substr(0,address.find("/"));
	std::string filepath = address.substr(package.length());
	std::string directory = ros::package::getPath(package);

	//Create new ros package if it doesn't already exist
	if(directory.compare("") == 0)
	{
		ROS_INFO("package \"%s\" not found. Creating a new package.", package.c_str());
		directory = generate_ros_package(package);
	}

	create_directories(directory, filepath.substr(0, filepath.find_last_of("/")));

	directory += filepath;

	//Write contents of request to file
	std::ofstream file_to_save;
	file_to_save.open (directory.c_str(), std::ios::binary);

	std::string str = "";
	for(int i=0; i < req.value.size(); i++){ str += req.value[i]; }
 	file_to_save << str;   
 	file_to_save.close();

	ROS_INFO("save_file request: %s", req.name.c_str());
	//ROS_INFO("package: %s\n",package.c_str());
	//ROS_INFO("filepath: %s\n",filepath.c_str());
	//ROS_INFO("directory: %s\n",directory.c_str());

	res.name = req.name;

	return true;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "file_server");
	ros::NodeHandle n;
	ros::ServiceServer serviceServer = n.advertiseService("/file_server/get_file", get_file);
	ros::ServiceServer serviceServer2 = n.advertiseService("/file_server/save_file", save_file);
	ROS_INFO("ROSbridge File Server initialized.");
	ros::spin();

	return 0;
}
