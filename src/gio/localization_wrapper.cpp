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
    lidar_odom_sub_ = nh.subscribe("lviorf/mapping/odometry", 10,  &LocalizationWrapper::LidarOdomCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("/fused_path", 10);
    pub_gps_imu_odometry_ = nh.advertise<nav_msgs::Odometry>("/gps_imu_odometry", 10);
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

    ConvertStateToOdom(fused_state);
    // timeInfoStamp = 
    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(ros_path_);

    //pub gps-imu-fused state.
    // nav_msgs::Odometry gps_imu_odometry_msg;
    // ConvertStateToOdometry(fused_state, gps_imu_odometry_msg);
    pub_gps_imu_odometry_.publish(curr_gps_odom);

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


void LocalizationWrapper::LidarOdomCallback(const nav_msgs::OdometryConstPtr& lidar_odom_msg_ptr) 
{
    double distance = 0.0;
    if(lidarOdomQueue.size() >= 2)
    {
    distance = sqrt(pow(lidarOdomQueue.back().pose.pose.position.x - lidarOdomQueue.front().pose.pose.position.x,2)+
                    pow(lidarOdomQueue.back().pose.pose.position.y - lidarOdomQueue.front().pose.pose.position.y,2)+
                    pow(lidarOdomQueue.back().pose.pose.position.z - lidarOdomQueue.front().pose.pose.position.z,2));
    }
    if(distance > 20.0)
    {
        return;
    }
    nav_msgs::Odometry lidar_odom = *lidar_odom_msg_ptr;
    lidarOdomQueue.push_back(lidar_odom);
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

void LocalizationWrapper::ConvertStateToOdom(const ImuGpsLocalization::State& state) 
{
    double distance = 0.0;
    if(gpsOdomQueue.size() >= 2)
    {
        distance = sqrt(pow(gpsOdomQueue.back().pose.pose.position.x - gpsOdomQueue.front().pose.pose.position.x,2)+
                           pow(gpsOdomQueue.back().pose.pose.position.y - gpsOdomQueue.front().pose.pose.position.y,2)+
                           pow(gpsOdomQueue.back().pose.pose.position.z - gpsOdomQueue.front().pose.pose.position.z,2));
    }
    if(distance > 20.0)
    {
        return;
    }
    nav_msgs::Odometry gps_odom_msg;
    gps_odom_msg.header.frame_id = "map";
    gps_odom_msg.child_frame_id = "gps";
    gps_odom_msg.header.stamp = ros::Time().fromSec(state.timestamp);
    gps_odom_msg.pose.pose.position.x = state.G_p_I[0];
    gps_odom_msg.pose.pose.position.y = state.G_p_I[1];
    gps_odom_msg.pose.pose.position.z = state.G_p_I[2];

    Eigen::Quaterniond G_q_I(state.G_R_I);
    gps_odom_msg.pose.pose.orientation.x = G_q_I.x();
    gps_odom_msg.pose.pose.orientation.y = G_q_I.y();
    gps_odom_msg.pose.pose.orientation.z = G_q_I.z();
    gps_odom_msg.pose.pose.orientation.w = G_q_I.w();

    gpsOdomQueue.push_back(gps_odom_msg);

}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "map";
    // ros_path_.header.stamp = ros::Time::now(); 
    ros_path_.header.stamp = ros::Time().fromSec(state.timestamp); 

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

    Eigen::Vector3d imu_position(state.G_p_I[0],state.G_p_I[1],state.G_p_I[2]);

    /*20250722将GPS位姿转换到Lidar坐标系下*/
    Eigen::Vector3d imu_2_lidar_T(-8.086759e-01, 3.195559e-01, -7.997231e-01);
    Eigen::Matrix3d imu_2_lidar_R;
    imu_2_lidar_R << 9.999976e-01, 7.553071e-04, -2.035826e-03,
                    -7.854027e-04, 9.998898e-01, -1.482298e-02,
                     2.024406e-03, 1.482454e-02, 9.998881e-01;

    // Eigen::Vector3d lidar_position = imu_2_lidar_R * imu_position + imu_2_lidar_T;
    // pose.pose.position.x = lidar_position[0];
    // pose.pose.position.y = lidar_position[1];
    // pose.pose.position.z = lidar_position[2];

    // Eigen::Quaterniond lidar_orientation = Eigen::Quaterniond(imu_2_lidar_R * G_q_I.toRotationMatrix());
    // pose.pose.orientation.x = lidar_orientation.x();
    // pose.pose.orientation.y = lidar_orientation.y();
    // pose.pose.orientation.z = lidar_orientation.z();
    // pose.pose.orientation.w = lidar_orientation.w();

    // // 定义欧拉角（以弧度为单位）
    // double roll = 0.0;   // 绕x轴旋转
    // double pitch = 0.0;  // 绕y轴旋转
    // double yaw = 0.0;  // 绕z轴旋转

    // double compute_yaw = 0.0;
    // if(ComputeRTbetweenLidarAndGPS(compute_yaw))
    // {
    //     yaw = compute_yaw;
    // }else{
    //     yaw = -M_PI / 2-0.165;  // 绕z轴旋转
    // }
    // yaw = 0.0;
    // // 将欧拉角转换为旋转矩阵
    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    //                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //                   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // Eigen::Vector3d lidar_position1 = rotation_matrix * imu_position;
    // pose.pose.position.x = lidar_position1[0];
    // pose.pose.position.y = lidar_position1[1];
    // pose.pose.position.z = lidar_position1[2];
    
    
    // Eigen::Quaterniond lidar_orientation1 = Eigen::Quaterniond(rotation_matrix * G_q_I.toRotationMatrix());
    // pose.pose.orientation.x = lidar_orientation1.x();
    // pose.pose.orientation.y = lidar_orientation1.y();
    // pose.pose.orientation.z = lidar_orientation1.z();
    // pose.pose.orientation.w = lidar_orientation1.w();           

    ros_path_.poses.push_back(pose);


    //写入里程计
    curr_gps_odom.header.frame_id = "map";
    curr_gps_odom.child_frame_id = "gps";
    curr_gps_odom.header.stamp = ros::Time().fromSec(state.timestamp);
    curr_gps_odom.pose.pose.position.x = pose.pose.position.x;
    curr_gps_odom.pose.pose.position.y = pose.pose.position.y;
    curr_gps_odom.pose.pose.position.z = pose.pose.position.z;
    curr_gps_odom.pose.pose.orientation.x = pose.pose.orientation.x;
    curr_gps_odom.pose.pose.orientation.y = pose.pose.orientation.y;
    curr_gps_odom.pose.pose.orientation.z = pose.pose.orientation.z;
    curr_gps_odom.pose.pose.orientation.w = pose.pose.orientation.w;

    //协方差
    curr_gps_odom.pose.covariance[0] = state.cov(0, 0);
    curr_gps_odom.pose.covariance[7] = state.cov(1, 1);
    curr_gps_odom.pose.covariance[14] = state.cov(2, 2);

}

void LocalizationWrapper::ConvertStateToOdometry(const ImuGpsLocalization::State& state,nav_msgs::Odometry& odom_msg) {
    
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "gps";
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

bool LocalizationWrapper::ComputeRTbetweenLidarAndGPS(double &yaw)
{
    // std::cout<<"gpsOdomQueue size: "<<gpsOdomQueue.size()<<",lidarOdomQueue size:"<<lidarOdomQueue.size()<<std::endl;
    if(gpsOdomQueue.size() < 2 || lidarOdomQueue.size() < 2 || is_get_yaw_between_lidar_gps)
     return false;
    if(lidarOdomQueue.size() >= 2 && gpsOdomQueue.back().header.stamp.toSec() >= lidarOdomQueue.back().header.stamp.toSec())
    {
        nav_msgs::Odometry gps_odom_0 = gpsOdomQueue.front();
        Eigen::Vector3d gps_position_0(gps_odom_0.pose.pose.position.x,gps_odom_0.pose.pose.position.y,gps_odom_0.pose.pose.position.z);
        nav_msgs::Odometry lidar_odom_0 = lidarOdomQueue.front();
        Eigen::Vector3d lidar_position_0(lidar_odom_0.pose.pose.position.x,lidar_odom_0.pose.pose.position.y,lidar_odom_0.pose.pose.position.z);
        
        nav_msgs::Odometry lidar_odom_1 = lidarOdomQueue.back();
        Eigen::Vector3d lidar_position_1(lidar_odom_1.pose.pose.position.x,lidar_odom_1.pose.pose.position.y,lidar_odom_1.pose.pose.position.z);

        nav_msgs::Odometry gps_odom_1;
        double diff_time_min = abs(gps_odom_0.header.stamp.toSec()-lidar_odom_1.header.stamp.toSec());
        for (auto it = gpsOdomQueue.begin(); it != gpsOdomQueue.end(); ++it) {
            double time_diff = abs(it->header.stamp.toSec() - lidar_odom_1.header.stamp.toSec());
            if (time_diff < diff_time_min) {
                diff_time_min = time_diff;
                gps_odom_1 = *it;
            }
        }

        //计算两个里程计之间的相对位姿
        Eigen::Vector3d gps_position_1(gps_odom_1.pose.pose.position.x,gps_odom_1.pose.pose.position.y,gps_odom_1.pose.pose.position.z);
        Eigen::Vector3d gps_position_diff = gps_position_1 - gps_position_0;
        Eigen::Vector3d lidar_position_diff = lidar_position_1 - lidar_position_0;

        yaw = std::acos(gps_position_diff.dot(lidar_position_diff) / (gps_position_diff.norm() * lidar_position_diff.norm()));
        std::cout<<"the yaw between lidar and gps is: "<<yaw<<" rad."<<std::endl; 
        is_get_yaw_between_lidar_gps = true;
    }
    return true;
}