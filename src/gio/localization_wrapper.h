#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#include "imu_gps_localizer.h"

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);
    void ConvertStateToOdometry(const ImuGpsLocalization::State& state,nav_msgs::Odometry& odom_msg);
    
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Publisher state_pub_;
    ros::Publisher pub_gps_imu_odometry_;

    std::ofstream file_state_;
    std::ofstream file_gps_;
    ImuGpsLocalization::State last_fused_state;

    nav_msgs::Path ros_path_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};