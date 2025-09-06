#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>
#include <deque>
#include <mutex>

class GNSSOdom
{
public:
    GNSSOdom(ros::NodeHandle &_nh) {
        nh = _nh;
        nh.param<std::string>("common/gps_topic", gps_topic, "/123");
        nh.param<std::string>("frame/map_frame", map_frame, "map");

        //读取原点经纬度坐标
        nh.param<double>("init_lla/initial_GPS_longitude", init_gps_longitude, 124.1);
        nh.param<double>("init_lla/initial_GPS_latitude", init_gps_latitude, 24.1);
        nh.param<double>("init_lla/initial_GPS_altitude", init_gps_altitude, 10);
        std::cout << "map_gps_longitude: " << init_gps_longitude << ", map_gps_latitude: " << init_gps_latitude << ", map_gps_altitude: " << init_gps_altitude << std::endl;

        std::cout << "gps topic: " << gps_topic << std::endl;

        gpsSub = nh.subscribe(gps_topic, 1000, &GNSSOdom::GNSSCB, this, ros::TransportHints().tcpNoDelay());
        lidar_odom_sub_ = nh.subscribe("lviorf/mapping/odometry", 10,  &GNSSOdom::LidarOdomCallback, this);

        left_odom_pub = nh.advertise<nav_msgs::Odometry>("/single_gps_odom", 100, false);
        init_origin_pub = nh.advertise<nav_msgs::Odometry>("/single_gps_init_odom", 10000, false);
        left_path_pub = nh.advertise<nav_msgs::Path>("/single_gps_path", 100);
    }

    bool ComputeRTbetweenLidarAndGPS()
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

        double yaw = std::acos(gps_position_diff.dot(lidar_position_diff) / (gps_position_diff.norm() * lidar_position_diff.norm()));
        std::cout<<"the yaw between lidar and gps is: "<<yaw<<" rad."<<std::endl; 



        // 定义欧拉角（以弧度为单位）
        double roll = 0.0;   // 绕x轴旋转
        double pitch = 0.0;  // 绕y轴旋转
        double dis_yaw = 0.0;  // 绕z轴旋转
        // 将欧拉角转换为旋转矩阵
        gps_to_lidar_rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        is_get_yaw_between_lidar_gps = true;
    }
    return true;
}

private:

    void LidarOdomCallback(const nav_msgs::OdometryConstPtr& lidar_odom_msg_ptr) 
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

    void AddGPStodeque(const nav_msgs::Odometry& gps_msg) 
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
        gps_odom_msg.header.stamp = gps_msg.header.stamp;
        gps_odom_msg.pose.pose.position.x = gps_msg.pose.pose.position.x;
        gps_odom_msg.pose.pose.position.y = gps_msg.pose.pose.position.y;
        gps_odom_msg.pose.pose.position.z = gps_msg.pose.pose.position.z;

        gps_odom_msg.pose.pose.orientation.x = gps_msg.pose.pose.orientation.x;
        gps_odom_msg.pose.pose.orientation.y = gps_msg.pose.pose.orientation.y;
        gps_odom_msg.pose.pose.orientation.z = gps_msg.pose.pose.orientation.z;
        gps_odom_msg.pose.pose.orientation.w = gps_msg.pose.pose.orientation.w;
        gpsOdomQueue.push_back(gps_odom_msg);

    }




    void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
        // std::cout << "gps status: " << msg->status.status << std::endl;
        if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
            return;
        }
        Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
        // std::cout << "LLA: " << lla.transpose() << std::endl;
        if (!initENU) {
            ROS_INFO("Init Orgin GPS latitude: %f, longitude:%f, altitude:%f", msg->latitude, msg->longitude, msg->altitude);
            geo_converter.Reset(lla[0], lla[1], lla[2]);
            initENU = true;


            /** publish initial pose from GNSS ENU Frame*/
            // nav_msgs::Odometry init_msg;
            // init_msg.header.stamp = msg->header.stamp;
            // init_msg.header.frame_id = map_frame;
            // init_msg.child_frame_id = "gps";
            // init_msg.pose.pose.position.x = lla[0];
            // init_msg.pose.pose.position.y = lla[1];
            // init_msg.pose.pose.position.z = lla[2];
            // init_msg.pose.covariance[0] = msg->position_covariance[0];
            // init_msg.pose.covariance[7] = msg->position_covariance[4];
            // init_msg.pose.covariance[14] = msg->position_covariance[8];
            // init_msg.pose.pose.orientation = yaw_quat_left;
            // init_origin_pub.publish(init_msg);
            // return;

            /***************************发布初始坐标在设置的全局坐标系中的位姿****************************** */
            double x, y, z;
            // // LLA->ENU, better accuacy than gpsTools especially for z value
            geo_converter.Forward(lla[0], lla[1], lla[2], x, y, z);
            Eigen::Vector3d enu(x, y, z);


            
            //求当前第一帧在全局坐标系中的位姿关系
            Eigen::Vector3d map_lla(init_gps_latitude, init_gps_longitude, init_gps_altitude);
            map_geo_converter.Reset(map_lla[0], map_lla[1], map_lla[2]);

            double map_x,map_y,map_z;
            map_geo_converter.Forward(lla[0], lla[1], lla[2], map_x, map_y, map_z);
            Eigen::Vector3d map_enu(map_x, map_y, map_z);

            Eigen::Vector3d init_position = enu - map_enu;
            ROS_INFO("init_position : %f, %f, %f", init_position(0), init_position(1), init_position(2));
        
            double init_yaw = atan2(enu(1) - map_enu(1), enu(0) - map_enu(0));
            ROS_INFO("init_yaw : %f", init_yaw);
            tf::Quaternion init_quat;
            init_quat.setRPY(0, 0, init_yaw);
            init_quat = init_quat.normalized();

            //发布TF
            geometry_msgs::TransformStamped init_tf;
            init_tf.header.stamp = msg->header.stamp;
            init_tf.header.frame_id = map_frame;
            init_tf.child_frame_id = "init_body";
            init_tf.transform.translation.x = map_x;
            init_tf.transform.translation.y = map_y;
            init_tf.transform.translation.z = map_z;
            init_tf.transform.rotation.x = init_quat.x();
            init_tf.transform.rotation.y = init_quat.y();
            init_tf.transform.rotation.z = init_quat.z();
            init_tf.transform.rotation.w = init_quat.w();
            init_tf_broadcaster.sendTransform(init_tf);

            /***************************结束发布初始坐标在设置的全局坐标系中的位姿****************************** */
            

            



            prev_pose_left = enu;
    
            nav_msgs::Odometry init_odom_msg;
            init_odom_msg.header.stamp = msg->header.stamp;
            init_odom_msg.header.frame_id = map_frame;
            init_odom_msg.child_frame_id = "imu_link";

            init_odom_msg.pose.pose.position.x = x;
            init_odom_msg.pose.pose.position.y = y;
            init_odom_msg.pose.pose.position.z = z;
            init_odom_msg.pose.pose.orientation.w = 1.0;

            AddGPStodeque(init_odom_msg);

            geometry_msgs::PoseStamped init_pose;
            init_pose.header = left_path.header;
            init_pose.pose.position.x = x;
            init_pose.pose.position.y = y;
            init_pose.pose.position.z = z;
            init_pose.pose.orientation.x = 0.0; 
            init_pose.pose.orientation.y = 0.0;
            init_pose.pose.orientation.z = 0.0;
            init_pose.pose.orientation.w = 1.0;
            left_path.poses.push_back(init_pose);
        }

        /** if you have some satellite info or rtk status info, put it here*/
        int status = -1;
        int satell_num = -1;
        double x, y, z;
        // LLA->ENU, better accuacy than gpsTools especially for z value
        geo_converter.Forward(lla[0], lla[1], lla[2], x, y, z);
        Eigen::Vector3d enu(x, y, z);
        // ROS_INFO("xyz : %f, %f, %f", enu(0), enu(1), enu(2));
        if (abs(enu.x()) > 10000 || abs(enu.x()) > 10000 || abs(enu.x()) > 10000) {
            /** check your lla coordinate */
            ROS_INFO("Error ogigin : %f, %f, %f", enu(0), enu(1), enu(2));
            return;
        }




        double yaw = 0.0;
        double distance = sqrt(pow(enu(1) - prev_pose_left(1), 2) + pow(enu(0) - prev_pose_left(0), 2));
        if (distance <= 10.0 && distance > 0.01) {
            // 返回值是此点与远点连线与x轴正方向的夹角
            yaw = atan2(enu(1) - prev_pose_left(1), enu(0) - prev_pose_left(0));
            yaw_quat_left = tf::createQuaternionMsgFromYaw(yaw);
            orientationReady = true;

            // std::cout << "gps odom yaw: " << yaw << std::endl;
        }
        
        // std::cout << "gps_odom lla: " << lla[0] << " " << lla[1] << " " << lla[2] << std::endl;

        if(!orientationReady)
        {
            ROS_WARN("waiting init origin yaw");
            prev_pose_left = enu;
            return;
        }
        if(distance > 10.0)
        {
            ROS_WARN("the distance between two gps pose is too large, distance = %f",distance);
            prev_pose_left = enu;
            return;
        }

        prev_pose_left = enu;

        /** pub gps odometry*/
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = map_frame;
        odom_msg.child_frame_id = "imu_link";

        odom_msg.pose.pose.position.x = enu(0);
        odom_msg.pose.pose.position.y = enu(1);
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.covariance[0] = msg->position_covariance[0];
        odom_msg.pose.covariance[7] = msg->position_covariance[4];
        odom_msg.pose.covariance[14] = msg->position_covariance[8];
        odom_msg.pose.covariance[1] = lla[0];
        odom_msg.pose.covariance[2] = lla[1];
        odom_msg.pose.covariance[3] = lla[2];
        odom_msg.pose.covariance[4] = status;
        odom_msg.pose.covariance[5] = satell_num;
        odom_msg.pose.covariance[6] = orientationReady; 
        odom_msg.pose.pose.orientation = yaw_quat_left; // ! 6轴imu，可以被用作粗糙的前端初始化

        AddGPStodeque(odom_msg);
        // std::cout<<"gpsOdomQueue.size = "<<gpsOdomQueue.size()<<",lidarOdomQueue.size = "<<lidarOdomQueue.size()<<std::endl;



       /*****************************20250729：进行位姿转换**************************************************** */
        if(!ComputeRTbetweenLidarAndGPS())
        {
            gps_to_lidar_rotation_matrix = Eigen::AngleAxisd(-M_PI / 2-0.165, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        }

        Eigen::Vector3d gps_position(enu(0),enu(1),enu(2));

        Eigen::Vector3d lidar_position = gps_to_lidar_rotation_matrix * gps_position;
        odom_msg.pose.pose.position.x = lidar_position[0];
        odom_msg.pose.pose.position.y = lidar_position[1];
        odom_msg.pose.pose.position.z = lidar_position[2];
        
        Eigen::Quaterniond eigen_q(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
                                   odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
                                    
        // 然后调用 toRotationMatrix()
        Eigen::Matrix3d Rwb = eigen_q.toRotationMatrix();

        Eigen::Quaterniond lidar_orientation1 = Eigen::Quaterniond(gps_to_lidar_rotation_matrix * Rwb);
        lidar_orientation1.normalize();
        odom_msg.pose.pose.orientation.x = lidar_orientation1.x();
        odom_msg.pose.pose.orientation.y = lidar_orientation1.y();
        odom_msg.pose.pose.orientation.z = lidar_orientation1.z();
        odom_msg.pose.pose.orientation.w = lidar_orientation1.w(); 

        // if(count%10 == 0) 
        //     ROS_INFO("gps_xyz : %f, %f, %f", lidar_position[0], lidar_position[1], lidar_position[2]);
       /*****************************20250729：位姿转换结束**************************************************** */
        double distance1 = sqrt(pow(lidar_position[0]-last_gps_position[0],2) + pow(lidar_position[1]-last_gps_position[1], 2));
        if (distance1 > 2.0 ) {
            last_gps_position = lidar_position;
            return;
        }
        left_odom_pub.publish(odom_msg);


        /** just for gnss visualization */
        // publish path
        left_path.header.frame_id = map_frame;
        left_path.header.stamp = msg->header.stamp;
        geometry_msgs::PoseStamped pose;
        pose.header = left_path.header;
        pose.pose.position.x = lidar_position[0];
        pose.pose.position.y = lidar_position[1];
        pose.pose.position.z = lidar_position[2];
        pose.pose.orientation.x = lidar_orientation1.x(); 
        pose.pose.orientation.y = lidar_orientation1.y();
        pose.pose.orientation.z = lidar_orientation1.z();
        pose.pose.orientation.w = lidar_orientation1.w();
        left_path.poses.push_back(pose);
        left_path_pub.publish(left_path); 

        last_gps_position = lidar_position;

        count++;
        if(count > 1000000) count = 0;
    }


    ros::NodeHandle nh;
    ros::Publisher left_odom_pub, left_path_pub, init_origin_pub;
    ros::Subscriber gpsSub;
    ros::Subscriber lidar_odom_sub_;

    std::mutex mutexLock;
    std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

    std::string gps_topic;
    std::string map_frame;

    bool initENU = false;
    bool orientationReady = false;
    nav_msgs::Path left_path;
    GeographicLib::LocalCartesian geo_converter;
    Eigen::Vector3d prev_pose_left, prev_pose_right;
    geometry_msgs::Quaternion yaw_quat_left;
    Eigen::Vector3d last_gps_position;

    std::deque<nav_msgs::Odometry> lidarOdomQueue; // 保存最近一段时间内的激光里程计消息
    std::deque<nav_msgs::Odometry> gpsOdomQueue; // 保存最近一段时间内的GPS里程计消息
    bool is_get_yaw_between_lidar_gps = false;
    Eigen::Matrix3d gps_to_lidar_rotation_matrix;

    int count = 0;
    GeographicLib::LocalCartesian map_geo_converter; // 用于转全局坐标的类
    double init_gps_longitude, init_gps_latitude, init_gps_altitude;

    tf::TransformBroadcaster init_tf_broadcaster;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "single_gps_odom");
    ros::NodeHandle nh;
    GNSSOdom gps(nh);
    ROS_INFO("\033[1;32m----> Simple GPS Odmetry Started.\033[0m");
    ros::spin();
    return 1;
}