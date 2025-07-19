#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>

#include "base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/imu_raw", 10,  &LocalizationWrapper::ImuCallback, this);
    gps_position_sub_ = nh.subscribe("/gps/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
    pub_gps_imu_odometry_ = nh.advertise<nav_msgs::Odometry>("gps_imu_odometry", 10);
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    if (!ok) {
        return;
    }

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(ros_path_);

    //pub gps-imu-fused state.
    nav_msgs::Odometry gps_imu_odometry_msg;
    ConvertStateToOdometry(fused_state, gps_imu_odometry_msg);
    pub_gps_imu_odometry_.publish(gps_imu_odometry_msg);

    last_fused_state = fused_state;


    // Log fused state.
    LogState(fused_state);

}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    if (gps_msg_ptr->status.status != 0) {
        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);

    LogGps(gps_data_ptr);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}

void LocalizationWrapper::ConvertStateToOdometry(const ImuGpsLocalization::State& state,nav_msgs::Odometry& odom_msg) {
    
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "gps_link";
    odom_msg.header.stamp = ros::Time::now();  

    odom_msg.pose.pose.position.x = state.G_p_I[0];
    odom_msg.pose.pose.position.y = state.G_p_I[1];
    odom_msg.pose.pose.position.z = state.G_p_I[2];

    // const Eigen::Quaterniond G_q_I(state.G_R_I);
    // odom_msg.pose.pose.orientation.x = G_q_I.x();
    // odom_msg.pose.pose.orientation.y = G_q_I.y();
    // odom_msg.pose.pose.orientation.z = G_q_I.z();
    // odom_msg.pose.pose.orientation.w = G_q_I.w();

    // odom_msg.pose.covariance[0] = state.cov(0, 0);
    // odom_msg.pose.covariance[7] = state.cov(1, 1);
    // odom_msg.pose.covariance[14] = state.cov(2, 2);
    // odom_msg.pose.covariance[21] = state.cov(3, 3);

    double distance = (state.G_p_I - last_fused_state.G_p_I).norm();
    // odom_msg.twist.twist.linear.x = distance / (state.timestamp - last_fused_state.timestamp);
    // odom_msg.twist.twist.linear.y = 0.0;
    // odom_msg.twist.twist.linear.z = 0.0;
    double yaw = 0.0;
    tf::Quaternion yaw_quat;
    if(distance > 0.1)
    {
        yaw = atan2(state.G_p_I[1] - last_fused_state.G_p_I[1], state.G_p_I[0] - last_fused_state.G_p_I[0]);
         yaw_quat = tf::createQuaternionFromYaw(yaw);
        // std::cout<<"gps_yaw: "<<yaw<<std::endl;
    }
    // odom_msg.pose.pose.orientation.x = yaw_quat.x();
    // odom_msg.pose.pose.orientation.y = yaw_quat.y();
    // odom_msg.pose.pose.orientation.z = yaw_quat.z();
    // odom_msg.pose.pose.orientation.w = yaw_quat.w();


    // Set the linear velocity
    // odom_msg.twist.twist.linear.x = state.G_v_I[0];
    // odom_msg.twist.twist.linear.y = state.G_v_I[1];
    // odom_msg.twist.twist.linear.z = state.G_v_I[2];
    
    // Set the angular velocity


}