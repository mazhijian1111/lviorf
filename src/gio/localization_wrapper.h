#pragma once

#include <fstream>
#include <memory>
#include <deque>

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

    void LidarOdomCallback(const nav_msgs::OdometryConstPtr& lidar_odom_msg_ptr); 

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);
    void ConvertStateToOdometry(const ImuGpsLocalization::State& state,nav_msgs::Odometry& odom_msg);
    void ConvertStateToOdom(const ImuGpsLocalization::State& state);
    bool ComputeRTbetweenLidarAndGPS(double &yaw);
    
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Subscriber lidar_odom_sub_;
    ros::Publisher state_pub_;
    ros::Publisher pub_gps_imu_odometry_;

    std::ofstream file_state_;
    std::ofstream file_gps_;
    ImuGpsLocalization::State last_fused_state;

    ros::Time timeInfoStamp;
    nav_msgs::Odometry curr_gps_odom;
    std::deque<nav_msgs::Odometry> lidarOdomQueue; // 保存最近一段时间内的激光里程计消息
    std::deque<nav_msgs::Odometry> gpsOdomQueue; // 保存最近一段时间内的GPS里程计消息
    bool is_get_yaw_between_lidar_gps = false;

    nav_msgs::Path ros_path_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};