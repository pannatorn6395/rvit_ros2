cmake_minimum_required(VERSION 3.5)

project(obo_rviz_util LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obo_sensor_msgs REQUIRED)
find_package(obo_nav_msgs REQUIRED)
find_package(topic_tools REQUIRED)
find_package(tf2_ros REQUIRED)
ament_package_xml()
set(ROS_DISTRO "ROS 2")
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")

set(${PROJECT_NAME}_headers_to_moc
        include/control_panel/control_panel.h
        include/control_panel/RosCommunication.h
        include/control_panel/CustomNavTool.h
        )


foreach(header "${${PROJECT_NAME}_headers_to_moc}")
    qt5_wrap_cpp(${PROJECT_NAME}_moc_files "${header}")
endforeach()

set(${PROJECT_NAME}_source_files
        src/control_panel.cpp
        include/control_panel/RosCommunication.cpp
        include/control_panel/RosCommunication.cpp
        )

add_library(${PROJECT_NAME} SHARED
        ${${PROJECT_NAME}_moc_files}
        ${${PROJECT_NAME}_source_files}
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
        Qt5::Widgets
        )

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        pluginlib
        rclcpp
        rviz_assimp_vendor
        rviz_rendering
        rviz_common
        geometry_msgs
	sensor_msgs
	obo_nav_msgs
	obo_sensor_msgs
	std_srvs
	topic_tools
	tf2_ros
	rviz_default_plugins
        )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
        rviz_rendering
        rviz_common
        rcl_interfaces
        pluginlib
        rclcpp
        std_msgs
        geometry_msgs
	sensor_msgs
	obo_nav_msgs
	obo_sensor_msgs
	std_srvs
	topic_tools
	tf2_ros
	rviz_default_plugins

)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_package()
