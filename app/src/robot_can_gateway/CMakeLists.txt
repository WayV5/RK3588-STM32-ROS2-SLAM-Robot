cmake_minimum_required(VERSION 3.8)
project(robot_can_gateway)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ---- Library: can_gateway_core ------------------------------------
add_library(can_gateway_core SHARED
	src/can_interface.cpp
	src/can_protocol.cpp
	src/kinematics.cpp
)
target_include_directories(can_gateway_core
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(can_gateway_core
	rclcpp::rclcpp
)

# ---- Executable: can_gateway_node ---------------------------------
add_executable(can_gateway_node
	src/can_gateway_node.cpp
)
ament_target_dependencies(can_gateway_node
	rclcpp
	geometry_msgs
	sensor_msgs
	nav_msgs
	diagnostic_msgs
	tf2
)
target_link_libraries(can_gateway_node
	can_gateway_core
)

# ---- Install rules -------------------------------------------------
install(TARGETS
	can_gateway_core
	can_gateway_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
