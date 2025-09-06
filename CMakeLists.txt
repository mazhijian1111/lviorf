cmake_minimum_required(VERSION 2.8.3)
project(lviorf)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  roslib
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geographic_msgs
  geodesy
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
# find_package(GeographicLib REQUIRED)

# 包含GeographicLib的头文件路径
include_directories("../third_party/GeographicLib")

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  tf2 tf2_ros geographic_msgs geodesy
)

add_subdirectory(third_party/GeographicLib)

# include directories
include_directories(
  lib
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  third_party/GeographicLib/include
)

# link directories
link_directories(
  lib
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########
# <!-- lviorf_yjz_lucky_boy -->
file(GLOB common_lib "lib/*.cpp")

# LIO
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/lio/imageProjection.cpp ${common_lib})
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/lio/mapOptmization.cpp ${common_lib})
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} libGeographiccc gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/lio/imuPreintegration.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)


######################
# <!-- lviorf_yjz_lucky_boy -->
# VIO
######################
file(GLOB visual_feature_files
    "src/vio/visual_feature/*.cpp"
    "src/vio/visual_feature/camera_models/*.cc"
)
file(GLOB visual_odometry_files
    "src/vio/visual_estimator/*.cpp"
    "src/vio/visual_estimator/factor/*.cpp"
    "src/vio/visual_estimator/initial/*.cpp"
    "src/vio/visual_estimator/utility/*.cpp"
)
file(GLOB visual_loop_files
    "src/vio/visual_loop/*.cpp"
    "src/vio/visual_loop/utility/*.cpp"
    "src/vio/visual_loop/ThirdParty/*.cpp"
    "src/vio/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/vio/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/vio/visual_loop/ThirdParty/DVision/*.cpp"
    "src/vio/visual_feature/camera_models/*.cc"
)
# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature ${visual_feature_files})
target_link_libraries(${PROJECT_NAME}_visual_feature ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
# Visual Odometry
add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
target_link_libraries(${PROJECT_NAME}_visual_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
# Visual Lopp
add_executable(${PROJECT_NAME}_visual_loop ${visual_loop_files})
target_link_libraries(${PROJECT_NAME}_visual_loop ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 


## GPS-IMU Localization node.
file(GLOB gps_imu_files
    "src/gio/gps_processor.cpp"
    "src/gio/imu_gps_localizer.cpp"
    "src/gio/imu_processor.cpp"
    "src/gio/initializer.cpp"
    "src/gio/localization_node.cpp"
    "src/gio/localization_wrapper.cpp"
)
add_executable(${PROJECT_NAME}_gps_imu_node ${gps_imu_files})
target_link_libraries(${PROJECT_NAME}_gps_imu_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
 glog 
 libGeographiccc
 #libGeographic
)

add_executable(${PROJECT_NAME}_gpsOdometry src/gio/simpleGpsOdom.cpp)
target_link_libraries(${PROJECT_NAME}_gpsOdometry ${catkin_LIBRARIES} libGeographiccc)


add_executable(global_fusion_node 
	src/loosen_fusion/global_fusion/globalOptNode.cpp
	src/loosen_fusion/global_fusion/globalOpt.cpp)

target_link_libraries(global_fusion_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} libGeographiccc) 




# GNSS里程计与激光里程计映射节点，将GNSS里程计映射到激光里程计上
add_executable(gnss_odom_calibrator src/gio/gnss_odom_calibrator.cpp)
target_link_libraries(gnss_odom_calibrator
  ${catkin_LIBRARIES} libGeographiccc
)

install(TARGETS gnss_odom_calibrator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# GNSS里程计、激光里程计、视觉里程计在后端进行松融合
add_executable(multi_sensor_fusion_node src/loosen_fusion/glvio_loosen/multi_sensor_fusion_node.cpp)
target_link_libraries(multi_sensor_fusion_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
install(TARGETS multi_sensor_fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)






