cmake_minimum_required(VERSION 3.5)
project(file_server2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add message and service dependencies
set(msg_dependencies
  std_msgs
)

set(srv_files 
  "srv/GetBinaryFile.srv"
  "srv/SaveBinaryFile.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES ${msg_dependencies}
)

# Build the executable
add_executable(file_server2_node src/file_server.cpp)

ament_target_dependencies(file_server2_node
  rclcpp
  std_msgs
)

# Get the typesupport targets
rosidl_get_typesupport_target(typesupport_target_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link the typesupport targets to your project
target_link_libraries(file_server2_node ${typesupport_target_cpp})

# rosidl_target_interfaces(file_server2_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install
install(TARGETS file_server2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install directories
install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install urdf file
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
ament_package()
