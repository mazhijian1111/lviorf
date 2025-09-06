#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <nav_msgs/Path.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>

class GNSSOdomCalibrator
{
public:
    GNSSOdomCalibrator() : nh_("~"), initialized_(false), min_samples_(100), origin_set_(false)
    {
        // Parameters
        nh_.param<std::string>("odom_topic", odom_topic_, "/laser_odom");
        nh_.param<std::string>("gnss_topic", gnss_topic_, "/gps/fix");
        nh_.param<std::string>("output_topic", output_topic_, "/gnss_calibrated");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<int>("min_samples", min_samples_, 100);
        nh_.param<double>("max_time_diff", max_time_diff_, 0.05);
        nh_.param<bool>("auto_set_origin", auto_set_origin_, true);
        nh_.param<double>("origin_lat", origin_lat_, 0.0);
        nh_.param<double>("origin_lon", origin_lon_, 0.0);
        nh_.param<double>("origin_alt", origin_alt_, 0.0);

        // Subscribers
        odom_sub_ = nh_.subscribe(odom_topic_, 100, &GNSSOdomCalibrator::odomCallback, this);
        gnss_sub_ = nh_.subscribe(gnss_topic_, 100, &GNSSOdomCalibrator::gnssCallback, this);

        // Publishers
        // calibrated_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_topic_, 10);
        calibed_gnss_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);
        enu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/gnss_enu", 10);
        transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/gnss_transform", 10);


        //for test
        lidar_path_pub_ = nh_.advertise<nav_msgs::Path>("/lidar_path_for_calib", 10);
        gnss_path_pub_ = nh_.advertise<nav_msgs::Path>("/gnss_path_for_calib", 10);
        calib_path_pub_ = nh_.advertise<nav_msgs::Path>("/gnss_path_calibed", 10);

        // Service to set origin manually
        // set_origin_srv_ = nh_.advertiseService("set_origin", &GNSSOdomCalibrator::setOriginCallback, this);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

        ROS_INFO("GNSS-Odom Calibrator initialized");
        ROS_INFO("Odom topic: %s", odom_topic_.c_str());
        ROS_INFO("GNSS topic: %s", gnss_topic_.c_str());
        ROS_INFO("Minimum samples: %d", min_samples_);

        setOrigin(origin_lat_, origin_lon_, origin_alt_);
        // if (!auto_set_origin_ && origin_lat_ != 0.0 && origin_lon_ != 0.0) {
        //     setOrigin(origin_lat_, origin_lon_, origin_alt_);
        // }
    }

private:
    struct PoseData
    {
        ros::Time stamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    struct GNSSData
    {
        ros::Time stamp;
        double latitude;
        double longitude;
        double altitude;
        Eigen::Vector3d enu_position;
    };

    // bool setOriginCallback(geographic_msgs::GeoPoint::Request& req,
    //                      geographic_msgs::GeoPoint::Response& res)
    // {
    //     setOrigin(req.latitude, req.longitude, req.altitude);
    //     res.success = true;
    //     res.message = "Origin set successfully";
    //     return true;
    // }

    void setOrigin(double lat, double lon, double alt)
    {
        origin_lat_ = lat;
        origin_lon_ = lon;
        origin_alt_ = alt;
        map_geo_converter.Reset(origin_lat_, origin_lon_, origin_alt_);
        origin_set_ = true;
        
        ROS_INFO("ENU origin set to: lat=%.8f, lon=%.8f, alt=%.3f", 
                 origin_lat_, origin_lon_, origin_alt_);
    }

    Eigen::Vector3d llhToEnu(double lat, double lon, double alt)
    {
        if (!origin_set_) {
            if (auto_set_origin_) {
                setOrigin(lat, lon, alt);
                return Eigen::Vector3d::Zero();
            } else {
                ROS_WARN_THROTTLE(1.0, "ENU origin not set, cannot convert coordinates");
                return Eigen::Vector3d::Zero();
            }
        }

        // Convert LLH to ECEF
        const double a = 6378137.0; // WGS84 semi-major axis
        const double e2 = 6.69437999014e-3; // WGS84 first eccentricity squared

        double sin_lat = sin(lat * M_PI / 180.0);
        double cos_lat = cos(lat * M_PI / 180.0);
        double sin_lon = sin(lon * M_PI / 180.0);
        double cos_lon = cos(lon * M_PI / 180.0);

        double N = a / sqrt(1 - e2 * sin_lat * sin_lat);
        double x = (N + alt) * cos_lat * cos_lon;
        double y = (N + alt) * cos_lat * sin_lon;
        double z = (N * (1 - e2) + alt) * sin_lat;

        // Convert origin to ECEF
        double sin_origin_lat = sin(origin_lat_ * M_PI / 180.0);
        double cos_origin_lat = cos(origin_lat_ * M_PI / 180.0);
        double sin_origin_lon = sin(origin_lon_ * M_PI / 180.0);
        double cos_origin_lon = cos(origin_lon_ * M_PI / 180.0);

        double N_origin = a / sqrt(1 - e2 * sin_origin_lat * sin_origin_lat);
        double x0 = (N_origin + origin_alt_) * cos_origin_lat * cos_origin_lon;
        double y0 = (N_origin + origin_alt_) * cos_origin_lat * sin_origin_lon;
        double z0 = (N_origin * (1 - e2) + origin_alt_) * sin_origin_lat;

        // Convert ECEF to ENU
        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        double east = -sin_origin_lon * dx + cos_origin_lon * dy;
        double north = -sin_origin_lat * cos_origin_lon * dx - sin_origin_lat * sin_origin_lon * dy + cos_origin_lat * dz;
        double up = cos_origin_lat * cos_origin_lon * dx + cos_origin_lat * sin_origin_lon * dy + sin_origin_lat * dz;

        return Eigen::Vector3d(east, north, up);
    }

    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
            ROS_WARN_THROTTLE(1.0, "GNSS fix not available");
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        GNSSData gnss_data;
        gnss_data.stamp = msg->header.stamp;
        gnss_data.latitude = msg->latitude;
        gnss_data.longitude = msg->longitude;
        gnss_data.altitude = msg->altitude;
        // gnss_data.enu_position = llhToEnu(msg->latitude, msg->longitude, msg->altitude);

        Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
        double map_x,map_y,map_z;
        map_geo_converter.Forward(lla[0], lla[1], lla[2], map_x, map_y, map_z);
        gnss_data.enu_position[0] = map_x;
        gnss_data.enu_position[1] = map_y;
        gnss_data.enu_position[2] = map_z;
        

        gnss_buffer_.push_back(gnss_data);

        // Keep buffer size manageable
        if (gnss_buffer_.size() > 1000) {
            gnss_buffer_.pop_front();
        }

        // Publish ENU coordinates for visualization
        geometry_msgs::PoseStamped enu_pose;
        enu_pose.header.stamp = msg->header.stamp;
        enu_pose.header.frame_id = "enu";
        enu_pose.pose.position.x = gnss_data.enu_position.x();
        enu_pose.pose.position.y = gnss_data.enu_position.y();
        enu_pose.pose.position.z = gnss_data.enu_position.z();
        enu_pose.pose.orientation.w = 1.0;
        enu_pub_.publish(enu_pose);

        // Try to process if we have odom data
        if (!odom_buffer_.empty()) {
            processData();
        }

        geometry_msgs::PoseStamped gnss_pose;
        gnss_pose.header.stamp = msg->header.stamp;
        gnss_pose.header.frame_id = odom_frame_;
        
        gnss_pose.pose.position.x = gnss_data.enu_position.x();
        gnss_pose.pose.position.y = gnss_data.enu_position.y();
        gnss_pose.pose.position.z = gnss_data.enu_position.z();
        
        gnss_pose.pose.orientation.w = 1.0;
        gnss_pose.pose.orientation.x = 0;
        gnss_pose.pose.orientation.y = 0;
        gnss_pose.pose.orientation.z = 0;
        gnss_path.header.frame_id = odom_frame_;
        gnss_path.header.stamp = msg->header.stamp;
        gnss_path.poses.push_back(gnss_pose);
        gnss_path_pub_.publish(gnss_path);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        PoseData odom_pose;
        odom_pose.stamp = msg->header.stamp;
        odom_pose.position = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        odom_pose.orientation = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );

        odom_buffer_.push_back(odom_pose);

        // Keep buffer size manageable
        if (odom_buffer_.size() > 1000) {
            odom_buffer_.pop_front();
        }

        // Try to process if we have GNSS data
        if (!gnss_buffer_.empty()) {
            processData();
        }

        geometry_msgs::PoseStamped lidar_pose;
        lidar_pose.header.stamp = msg->header.stamp;
        lidar_pose.header.frame_id = odom_frame_;
        
        lidar_pose.pose.position.x = msg->pose.pose.position.x;
        lidar_pose.pose.position.y = msg->pose.pose.position.y;
        lidar_pose.pose.position.z = msg->pose.pose.position.z;
        
        lidar_pose.pose.orientation.w = msg->pose.pose.orientation.w;
        lidar_pose.pose.orientation.x = msg->pose.pose.orientation.x;
        lidar_pose.pose.orientation.y = msg->pose.pose.orientation.y;
        lidar_pose.pose.orientation.z = msg->pose.pose.orientation.z;
        lidar_path.header.frame_id = odom_frame_;
        lidar_path.header.stamp = msg->header.stamp;
        lidar_path.poses.push_back(lidar_pose);
        lidar_path_pub_.publish(lidar_path);
    }

    void processData()
    {
        if (initialized_) {
            // Already calibrated, just transform new GNSS data
            transformLatestGNSS();
            return;
        }

        if (!origin_set_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for ENU origin to be set");
            return;
        }

        // Collect synchronized data pairs for calibration
        std::vector<Eigen::Vector3d> odom_positions;
        std::vector<Eigen::Vector3d> gnss_enu_positions;

        for (const auto& gnss_data : gnss_buffer_) {
            // Find closest odom data
            auto odom_iter = findClosestOdom(gnss_data.stamp);
            if (odom_iter != odom_buffer_.end()) {
                double time_diff = std::abs((gnss_data.stamp - odom_iter->stamp).toSec());
                if (time_diff < max_time_diff_) {
                    odom_positions.push_back(odom_iter->position);
                    gnss_enu_positions.push_back(gnss_data.enu_position);
                }
            }
        }

        if (odom_positions.size() >= min_samples_) {
            if (calibrateTransform(odom_positions, gnss_enu_positions)) {
                initialized_ = true;
                ROS_INFO("Calibration completed successfully with %zu samples!", odom_positions.size());
                ROS_INFO("Translation: [%.3f, %.3f, %.3f]", 
                         transform_translation_.x(), 
                         transform_translation_.y(), 
                         transform_translation_.z());
                
                Eigen::Vector3d euler = transform_rotation_.matrix().eulerAngles(0, 1, 2);
                ROS_INFO("Rotation (RPY): [%.3f, %.3f, %.3f] radians",
                         euler.x(), euler.y(), euler.z());
            }
        } else if (odom_positions.size() > 10) {
            ROS_INFO_THROTTLE(5.0, "Collecting calibration data: %zu/%d samples", 
                             odom_positions.size(), min_samples_);
        }
    }

    bool calibrateTransform(const std::vector<Eigen::Vector3d>& odom_positions,
                           const std::vector<Eigen::Vector3d>& gnss_positions)
    {
        if (odom_positions.size() != gnss_positions.size() || odom_positions.size() < 3) {
            ROS_ERROR("Not enough data points for calibration");
            return false;
        }

        try {
            // Compute centroids
            Eigen::Vector3d centroid_odom = Eigen::Vector3d::Zero();
            Eigen::Vector3d centroid_gnss = Eigen::Vector3d::Zero();

            for (size_t i = 0; i < odom_positions.size(); ++i) {
                centroid_odom += odom_positions[i];
                centroid_gnss += gnss_positions[i];
            }

            centroid_odom /= odom_positions.size();
            centroid_gnss /= gnss_positions.size();

            // Compute covariance matrix
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            for (size_t i = 0; i < odom_positions.size(); ++i) {
                Eigen::Vector3d odom_centered = odom_positions[i] - centroid_odom;
                Eigen::Vector3d gnss_centered = gnss_positions[i] - centroid_gnss;
                H += odom_centered * gnss_centered.transpose();
            }

            // SVD decomposition
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d U = svd.matrixU();
            Eigen::Matrix3d V = svd.matrixV();

            // Compute rotation
            Eigen::Matrix3d R = V * U.transpose();

            // Handle reflection case
            if (R.determinant() < 0) {
                V.col(2) *= -1;
                R = V * U.transpose();
            }

            // Compute translation
            Eigen::Vector3d t = centroid_gnss - R * centroid_odom;

            // Convert to quaternion
            transform_rotation_ = Eigen::Quaterniond(R);
            transform_translation_ = t;

            // Calculate calibration error
            double error = calculateCalibrationError(odom_positions, gnss_positions, R, t);
            ROS_INFO("Calibration RMS error: %.3f meters", error);

            if (error > 2.0) {
                ROS_WARN("High calibration error detected, results may be unreliable");
            }

            return true;

        } catch (const std::exception& e) {
            ROS_ERROR("Calibration failed: %s", e.what());
            return false;
        }
    }

    double calculateCalibrationError(const std::vector<Eigen::Vector3d>& odom_positions,
                                    const std::vector<Eigen::Vector3d>& gnss_positions,
                                    const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
    {
        double total_error = 0.0;
        for (size_t i = 0; i < odom_positions.size(); ++i) {
            Eigen::Vector3d transformed = R * odom_positions[i] + t;
            double error = (transformed - gnss_positions[i]).norm();
            total_error += error * error;
        }
        return sqrt(total_error / odom_positions.size());
    }

    void transformLatestGNSS()
    {
        if (gnss_buffer_.empty()) return;

        const auto& latest_gnss = gnss_buffer_.back();
        
        // Transform GNSS ENU position to odom frame
        // Eigen::Vector3d transformed_pos = transform_rotation_.inverse() * latest_gnss.enu_position + transform_translation_;
        Eigen::Vector3d transformed_pos = transform_rotation_.inverse() * latest_gnss.enu_position;

        // Create output message
        geometry_msgs::PoseStamped calibrated_pose;
        calibrated_pose.header.stamp = latest_gnss.stamp;
        calibrated_pose.header.frame_id = odom_frame_;
        
        calibrated_pose.pose.position.x = transformed_pos.x();
        calibrated_pose.pose.position.y = transformed_pos.y();
        calibrated_pose.pose.position.z = 0.0;
        
        // Use identity orientation for now (or could use transformed orientation if available)
        calibrated_pose.pose.orientation.w = 1.0;
        calibrated_pose.pose.orientation.x = 0.0;
        calibrated_pose.pose.orientation.y = 0.0;
        calibrated_pose.pose.orientation.z = 0.0;

        // Publish calibrated pose
        // calibrated_pub_.publish(calibrated_pose);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = latest_gnss.stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = "gnss_calibrated";
        odom_msg.pose.pose.position.x = transformed_pos.x();
        odom_msg.pose.pose.position.y = transformed_pos.y();
        odom_msg.pose.pose.position.z = 0.0;
        calibed_gnss_pub_.publish(odom_msg);


        // Publish transform
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = "gnss_calibrated";
        
        transform.transform.translation.x = transform_translation_.x();
        transform.transform.translation.y = transform_translation_.y();
        transform.transform.translation.z = transform_translation_.z();
        
        transform.transform.rotation.w = transform_rotation_.w();
        transform.transform.rotation.x = transform_rotation_.x();
        transform.transform.rotation.y = transform_rotation_.y();
        transform.transform.rotation.z = transform_rotation_.z();

        transform_pub_.publish(transform);
        tf_broadcaster_->sendTransform(transform);

        calibrated_pose.pose.position.z = 0.0;
        calib_path.header.frame_id = odom_frame_;
        calib_path.header.stamp = latest_gnss.stamp;
        calib_path.poses.push_back(calibrated_pose);
        calib_path_pub_.publish(calib_path);


    }

    std::deque<PoseData>::const_iterator findClosestOdom(const ros::Time& stamp)
    {
        if (odom_buffer_.empty()) return odom_buffer_.end();

        auto closest = odom_buffer_.begin();
        double min_diff = std::abs((stamp - closest->stamp).toSec());

        for (auto it = odom_buffer_.begin(); it != odom_buffer_.end(); ++it) {
            double diff = std::abs((stamp - it->stamp).toSec());
            if (diff < min_diff) {
                min_diff = diff;
                closest = it;
            }
        }

        return closest;
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, gnss_sub_;
    ros::Publisher calibrated_pub_, enu_pub_, transform_pub_;

    ros::Publisher lidar_path_pub_,gnss_path_pub_,calib_path_pub_,calibed_gnss_pub_;
    nav_msgs::Path lidar_path,gnss_path,calib_path;
    // ros::ServiceServer set_origin_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::deque<PoseData> odom_buffer_;
    std::deque<GNSSData> gnss_buffer_;
    std::mutex mutex_;

    Eigen::Vector3d transform_translation_;
    Eigen::Quaterniond transform_rotation_;
    bool initialized_;
    bool origin_set_;
    bool auto_set_origin_;
    double origin_lat_, origin_lon_, origin_alt_;

    std::string odom_topic_, gnss_topic_, output_topic_;
    std::string odom_frame_, base_frame_;
    int min_samples_;
    double max_time_diff_;
    GeographicLib::LocalCartesian map_geo_converter; // 用于转全局坐标的类
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_odom_calibrator");
    ROS_INFO("\033[1;32m----> gnss_odom_calibrator Started.\033[0m");
    
    GNSSOdomCalibrator calibrator;
    ros::spin();
    
    return 0;
}