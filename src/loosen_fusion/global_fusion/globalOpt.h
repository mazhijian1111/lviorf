/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"
#include "ros/ros.h"

using namespace std;

// 声明外部变量
// extern double ori_lat;
// extern double ori_lon;
// extern double ori_alt;
// double ori_lat;
// double ori_lon;
// double ori_alt;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputGPSOdom(double t, nav_msgs::OdometryConstPtr GPS_msg);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void inputLidarOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::Path global_path;

	nav_msgs::Path gnss_path;
	// static double ori_lat;
	// static double ori_lon;
	// static double ori_alt;

private:
	void setOrigin(double lat, double lon, double alt);
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();
	void updateGlobalPath();

	// ros::NodeHandle nh;

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap,globalLIOPoseMap;
	map<double, vector<double>> GPSPositionMap;
	map<double, vector<double>> localLidarPoseMap;

	bool initGPS;
	bool newGPS;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap,mPoseMap1;
	Eigen::Matrix4d WGPS_T_WVIO,WGPS_T_WLIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	Eigen::Vector3d LIO_lastP;
	Eigen::Quaterniond LIO_lastQ;
	std::thread threadOpt;
	double init_gps_longitude, init_gps_latitude, init_gps_altitude;
	

};