cmake_minimum_required(VERSION 3.0.2)

project(ainex_gazebo)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
