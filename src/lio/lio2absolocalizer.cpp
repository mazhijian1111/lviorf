#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <Eigen/Dense>
#include "LocalCartesian.hpp"

class LaserOdomToGlobalENU
{
public:
    LaserOdomToGlobalENU() : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 参数配置
        private_nh.param("odom_frame", odom_frame_, std::string("odom"));
        private_nh.param("base_frame", base_frame_, std::string("base_link"));
        private_nh.param("global_frame", global_frame_, std::string("map"));
        private_nh.param("gnss_topic", gnss_topic_, std::string("/gnss/fix"));
        private_nh.param("laser_odom_topic", laser_odom_topic_, std::string("/laser_odom"));
        private_nh.param("initial_latitude", init_gps_latitude, 0.0);
        private_nh.param("initial_longitude", init_gps_longitude, 0.0);
        private_nh.param("initial_altitude", init_gps_altitude, 0.0);
        
        
        // 订阅和发布
        gnss_sub_ = nh.subscribe(gnss_topic_, 10, &LaserOdomToGlobalENU::gnssCallback, this);
        laser_odom_sub_ = nh.subscribe(laser_odom_topic_, 10, &LaserOdomToGlobalENU::laserOdomCallback, this);
        global_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/global_odom", 10);
        
        // 初始化状态
        first_gnss_received_ = false;
        first_odom_received_ = false;
        initial_gnss_set_ = false;

        if(init_gps_latitude+init_gps_longitude+init_gps_altitude != 0.0)
        {
            setOrigin(init_gps_latitude, init_gps_longitude, init_gps_altitude);
        }
        
        ROS_INFO("LaserOdomToGlobalENU initialized");
    }


    void setOrigin(double lat, double lon, double alt)
    {
        geoConverter.Reset(lat, lon, alt);
        first_gnss_received_ = true;
        ROS_INFO("fusion node ENU origin set to: lat=%.8f, lon=%.8f, alt=%.3f", lat, lon, alt);
    }

    /**
     * @brief GNSS回调函数，记录第一帧GNSS数据
     */
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (!first_gnss_received_ && msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX)
        {
            initial_gnss_ = *msg;
            geoConverter.Reset(initial_gnss_.latitude, initial_gnss_.longitude, initial_gnss_.altitude);
            first_gnss_received_ = true;
            ROS_INFO("First GNSS frame received: lat=%.6f, lon=%.6f, alt=%.3f", 
                     initial_gnss_.latitude, initial_gnss_.longitude, initial_gnss_.altitude);
        }
    }

    /**
     * @brief 激光里程计回调函数，进行坐标转换
     */
    void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (!first_gnss_received_)
        {
            ROS_WARN_THROTTLE(1.0, "Waiting for first GNSS data...");
            return;
        }

        if (!first_odom_received_)
        {
            // 记录第一帧里程计位姿
            initial_odom_ = *msg;
            first_odom_received_ = true;
            
            // 设置初始GNSS位姿（转换为ENU坐标系）
            if (!setInitialGNSSPose())
            {
                ROS_WARN_THROTTLE(1.0, "Failed to set initial GNSS pose");
                return;
            }
            
            ROS_INFO("First odometry received, initial transform set");
        }

        // 转换当前里程计到全局ENU坐标系
        nav_msgs::Odometry global_odom;
        if (convertToENUFrame(*msg, global_odom))
        {
            global_odom_pub_.publish(global_odom);
            publishGlobalTransform(global_odom);
        }
    }

private:
    /**
     * @brief 将WGS84经纬高转换为ENU坐标系
     * @param lat 纬度 (degrees)
     * @param lon 经度 (degrees)
     * @param alt 高度 (meters)
     * @param ref_lat 参考点纬度
     * @param ref_lon 参考点经度
     * @param ref_alt 参考点高度
     * @param enu 输出的ENU坐标 [east, north, up]
     */
    void wgs84ToENU(double lat, double lon, double alt, 
                   double ref_lat, double ref_lon, double ref_alt,
                   double enu[3])
    {
        // 将角度转换为弧度
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        double ref_lat_rad = ref_lat * M_PI / 180.0;
        double ref_lon_rad = ref_lon * M_PI / 180.0;
        
        // WGS84椭球参数
        const double a = 6378137.0;        // 半长轴
        const double b = 6356752.314245;   // 半短轴
        const double e_sq = 1 - (b * b) / (a * a); // 第一偏心率平方
        
        // 计算参考点的地心坐标
        double N_ref = a / sqrt(1 - e_sq * sin(ref_lat_rad) * sin(ref_lat_rad));
        double x_ref = (N_ref + ref_alt) * cos(ref_lat_rad) * cos(ref_lon_rad);
        double y_ref = (N_ref + ref_alt) * cos(ref_lat_rad) * sin(ref_lon_rad);
        double z_ref = (N_ref * (1 - e_sq) + ref_alt) * sin(ref_lat_rad);
        
        // 计算当前点的地心坐标
        double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));
        double x = (N + alt) * cos(lat_rad) * cos(lon_rad);
        double y = (N + alt) * cos(lat_rad) * sin(lon_rad);
        double z = (N * (1 - e_sq) + alt) * sin(lat_rad);
        
        // 计算ENU坐标
        double dx = x - x_ref;
        double dy = y - y_ref;
        double dz = z - z_ref;
        
        // ENU转换矩阵
        enu[0] = -sin(ref_lon_rad) * dx + cos(ref_lon_rad) * dy;
        enu[1] = -sin(ref_lat_rad) * cos(ref_lon_rad) * dx - sin(ref_lat_rad) * sin(ref_lon_rad) * dy + cos(ref_lat_rad) * dz;
        enu[2] = cos(ref_lat_rad) * cos(ref_lon_rad) * dx + cos(ref_lat_rad) * sin(ref_lon_rad) * dy + sin(ref_lat_rad) * dz;
    }


        //将GPS经纬高转为xyz
    void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
    {
        if(!initGPS)
        {
            geoConverter.Reset(latitude, longitude, altitude);
            printf("fusion node ENU origin: la: %f lo: %f al: %f\n", latitude, longitude, altitude);
            initGPS = true;
        }
        geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
        //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
        //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
    }

    /**
     * @brief 设置初始GNSS位姿（转换为ENU坐标系）
     */
    bool setInitialGNSSPose()
    {
        try
        {
            // 将第一帧GNSS作为ENU坐标系原点
            double enu[3] = {0, 0, 0};
            wgs84ToENU(initial_gnss_.latitude, initial_gnss_.longitude, initial_gnss_.altitude,
                      initial_gnss_.latitude, initial_gnss_.longitude, initial_gnss_.altitude,
                      enu);
            
            // 创建初始GNSS位姿（在map坐标系下）
            initial_gnss_pose_.header.stamp = initial_gnss_.header.stamp;
            initial_gnss_pose_.header.frame_id = global_frame_;
            initial_gnss_pose_.child_frame_id = base_frame_;
            
            initial_gnss_pose_.pose.pose.position.x = enu[0];  // East
            initial_gnss_pose_.pose.pose.position.y = enu[1];  // North
            initial_gnss_pose_.pose.pose.position.z = enu[2];  // Up
            
            // 初始朝向（假设为0，实际可能需要从IMU获取）
            initial_gnss_pose_.pose.pose.orientation.w = 1.0;
            initial_gnss_pose_.pose.pose.orientation.x = 0.0;
            initial_gnss_pose_.pose.pose.orientation.y = 0.0;
            initial_gnss_pose_.pose.pose.orientation.z = 0.0;
            
            // 保存参考点经纬高
            ref_lat_ = initial_gnss_.latitude;
            ref_lon_ = initial_gnss_.longitude;
            ref_alt_ = initial_gnss_.altitude;
            
            initial_gnss_set_ = true;
            ROS_INFO("Initial GNSS pose set in ENU: E=%.3f, N=%.3f, U=%.3f", 
                     enu[0], enu[1], enu[2]);
            
            return true;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Failed to set initial GNSS pose: %s", e.what());
            return false;
        }
    }

    /**
     * @brief 将激光里程计转换为ENU全局坐标系
     * @param laser_odom 输入的激光里程计
     * @param global_odom 输出的全局位姿
     */
    bool convertToENUFrame(const nav_msgs::Odometry& laser_odom, nav_msgs::Odometry& global_odom)
    {
        if (!initial_gnss_set_)
        {
            ROS_WARN("Initial GNSS pose not set");
            return false;
        }

        try
        {
            // 计算相对变换：current_odom * initial_odom^{-1}
            tf2::Transform initial_odom_tf, current_odom_tf, relative_tf;
            
            // 初始里程计位姿
            tf2::fromMsg(initial_odom_.pose.pose, initial_odom_tf);
            
            // 当前里程计位姿
            tf2::fromMsg(laser_odom.pose.pose, current_odom_tf);
            
            // 相对变换：从初始到当前的运动
            relative_tf = initial_odom_tf.inverse() * current_odom_tf;
            
            // 应用到初始GNSS位姿：global_pose = initial_gnss_pose * relative_tf
            tf2::Transform initial_gnss_tf;
            tf2::fromMsg(initial_gnss_pose_.pose.pose, initial_gnss_tf);
            
            tf2::Transform global_tf = initial_gnss_tf * relative_tf;
            
            // 设置输出
            global_odom = laser_odom;
            global_odom.header.stamp = laser_odom.header.stamp;
            global_odom.header.frame_id = global_frame_;
            global_odom.child_frame_id = base_frame_;
            
            global_odom.pose.pose.position.x = global_tf.getOrigin().x();
            global_odom.pose.pose.position.y = global_tf.getOrigin().y();
            global_odom.pose.pose.position.z = global_tf.getOrigin().z();
            global_odom.pose.pose.orientation = tf2::toMsg(global_tf.getRotation());
            
            // 转换协方差矩阵（如果需要）
            // 这里假设协方差矩阵在ENU坐标系下保持不变
            
            return true;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Failed to convert to ENU frame: %s", e.what());
            return false;
        }
    }

    /**
     * @brief 发布全局坐标系下的tf变换
     */
    void publishGlobalTransform(const nav_msgs::Odometry& global_odom)
    {
        geometry_msgs::TransformStamped transform;
        transform.header = global_odom.header;
        transform.child_frame_id = global_odom.child_frame_id;
        
        transform.transform.translation.x = global_odom.pose.pose.position.x;
        transform.transform.translation.y = global_odom.pose.pose.position.y;
        transform.transform.translation.z = global_odom.pose.pose.position.z;
        transform.transform.rotation = global_odom.pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(transform);
    }

    bool initGPS;
    GeographicLib::LocalCartesian geoConverter;
    double init_gps_longitude, init_gps_latitude, init_gps_altitude;

    // ROS相关
    ros::Subscriber gnss_sub_;
    ros::Subscriber laser_odom_sub_;
    ros::Publisher global_odom_pub_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 参数
    std::string odom_frame_;
    std::string base_frame_;
    std::string global_frame_;
    std::string gnss_topic_;
    std::string laser_odom_topic_;
    
    // 状态标志
    bool first_gnss_received_;
    bool first_odom_received_;
    bool initial_gnss_set_;
    
    // 初始数据和参考点
    sensor_msgs::NavSatFix initial_gnss_;
    nav_msgs::Odometry initial_odom_;
    nav_msgs::Odometry initial_gnss_pose_;
    double ref_lat_, ref_lon_, ref_alt_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_odom_to_global_enu");
    LaserOdomToGlobalENU converter;
    ros::spin();
    return 0;
}