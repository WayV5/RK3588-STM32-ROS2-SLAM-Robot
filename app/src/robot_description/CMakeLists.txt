cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config worlds
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
