cmake_minimum_required(VERSION 2.8.3)

<<<<<<< HEAD
project(bimanual_arm)
=======
project(armDouble.SLDASM)
>>>>>>> 00aee69e (improve-badminton-rl)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
