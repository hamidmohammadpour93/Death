cmake_minimum_required(VERSION 3.5)
project(tb4_kostal_gui LANGUAGES CXX)

# Find packages and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tb4_kostal_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Add executable
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp tb4_kostal_msgs irobot_create_msgs rclcpp_action)

add_executable(set_grid_client
    src/set_grid_client.cpp
)
ament_target_dependencies(set_grid_client
    rclcpp tb4_kostal_msgs irobot_create_msgs rclcpp_action ament_index_cpp
    control_msgs geometry_msgs
)

add_executable(set_grid
    src/set_grid.cpp
)
ament_target_dependencies(set_grid
    rclcpp tb4_kostal_msgs irobot_create_msgs rclcpp_action ament_index_cpp
    control_msgs geometry_msgs
)

add_executable(set_grid_2d
    src/set_grid_2d.cpp
)
ament_target_dependencies(set_grid_2d
    rclcpp tb4_kostal_msgs irobot_create_msgs rclcpp_action ament_index_cpp
    control_msgs geometry_msgs
)

add_executable(grid_server src/grid_server.cpp)
ament_target_dependencies(grid_server rclcpp tb4_kostal_msgs irobot_create_msgs rclcpp_action)

# Link against libraries
target_link_libraries(set_grid_client
    ${rclcpp_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
)
target_link_libraries(set_grid
    ${rclcpp_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
)
target_link_libraries(set_grid_2d
    ${rclcpp_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
)

# Install executable
install(TARGETS
    action_client
    set_grid_client
    set_grid
    set_grid_2d
    grid_server
    DESTINATION lib/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY resource
   DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
ament_package()
