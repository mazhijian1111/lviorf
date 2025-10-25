#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <deque>
#include <mutex>

class MultiOdomPublisher {
private:
    ros::NodeHandle nh_;
    
    // 订阅者
    ros::Subscriber gps_odom_sub_;
    ros::Subscriber laser_odom_sub_;
    
    // 发布者
    ros::Publisher transformed_gps_pub_;
    ros::Publisher original_laser_pub_;
    ros::Publisher original_gps_pub_;
    ros::Publisher all_odom_pub_;  // 可选：发布所有里程计的数组
    ros::Publisher transformed_gps_path_pub_;
    ros::Publisher original_laser_path_pub_;
    ros::Publisher original_gps_path_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::Path transformed_gps_path;
    nav_msgs::Path original_laser_path;
    nav_msgs::Path original_gps_path;

    // 数据队列
    std::deque<std::pair<ros::Time, Eigen::Isometry3d>> gps_odom_queue_;
    std::deque<std::pair<ros::Time, Eigen::Isometry3d>> laser_odom_queue_;
    
    // 参数
    int max_queue_size_;
    double max_time_diff_;
    double min_data_points_;
    
    // 变换矩阵
    Eigen::Isometry3d gps_to_laser_transform_;
    bool transform_computed_;
    std::mutex mutex_;
    
    // 坐标系名称
    std::string laser_odom_frame_;
    std::string gps_odom_frame_;
    std::string transformed_gps_frame_;
    std::string base_frame_;

public:
    MultiOdomPublisher() : transform_computed_(false) {
        // 初始化参数
        nh_.param<int>("max_queue_size", max_queue_size_, 100000);
        nh_.param<double>("max_time_diff", max_time_diff_, 0.1);
        nh_.param<double>("min_data_points", min_data_points_, 100);
        nh_.param<std::string>("laser_odom_frame", laser_odom_frame_, "map");
        nh_.param<std::string>("gps_odom_frame", gps_odom_frame_, "map");
        nh_.param<std::string>("transformed_gps_frame", transformed_gps_frame_, "map");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        
        // 初始化变换矩阵为单位矩阵
        gps_to_laser_transform_.setIdentity();
        
        // 初始化发布者和订阅者
        gps_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/lviorf/mapping/gps_odom", 10, &MultiOdomPublisher::gpsOdomCallback, this);
        
        laser_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/lviorf/mapping/odometry", 10, &MultiOdomPublisher::laserOdomCallback, this);
        
        // 初始化多个发布者
        transformed_gps_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom_transformed", 10);
        original_laser_pub_ = nh_.advertise<nav_msgs::Odometry>("/laser_odom_original", 10);
        original_gps_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom_original", 10);
        all_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/all_odom_comparison", 10);

        transformed_gps_path_pub_= nh_.advertise<nav_msgs::Path>("/transformed_gps_path", 10);
        original_laser_path_pub_= nh_.advertise<nav_msgs::Path>("/original_laser_path", 10);
        original_gps_path_pub_= nh_.advertise<nav_msgs::Path>("/original_gps_path", 10);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
        
        ROS_INFO("MultiOdomPublisher initialized");
        ROS_INFO("Publishing topics:");
        ROS_INFO("  - /gps_odom_transformed: GPS odometry in laser frame");
        ROS_INFO("  - /laser_odom_original: Original laser odometry");
        ROS_INFO("  - /gps_odom_original: Original GPS odometry");
        ROS_INFO("  - /all_odom_comparison: All odometry for comparison");
    }

private:
    void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 发布原始 GPS 里程计
        publishOriginalGpsOdometry(*msg);
        
        // 转换到 Eigen 格式并添加到队列
        Eigen::Isometry3d pose = odometryToIsometry(*msg);
        gps_odom_queue_.push_back(std::make_pair(msg->header.stamp, pose));
        
        // 维护队列大小
        if (gps_odom_queue_.size() > max_queue_size_) {
            gps_odom_queue_.pop_front();
        }
        
        // 尝试计算变换矩阵
        if (!transform_computed_) {
            computeTransform();
        }
        
        // 如果变换已计算，发布变换后的 GPS 里程计
        if (transform_computed_) {
            publishTransformedGpsOdometry(*msg);
            publishAllOdometryComparison(*msg);
        }
    }
    
    void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 发布原始激光里程计
        publishOriginalLaserOdometry(*msg);
        
        // 转换到 Eigen 格式并添加到队列
        Eigen::Isometry3d pose = odometryToIsometry(*msg);
        laser_odom_queue_.push_back(std::make_pair(msg->header.stamp, pose));
        
        // 维护队列大小
        if (laser_odom_queue_.size() > max_queue_size_) {
            laser_odom_queue_.pop_front();
        }
        
        // 尝试计算变换矩阵
        if (!transform_computed_) {
            computeTransform();
        }
    }
    
    void computeTransform() {
        if (gps_odom_queue_.size() < min_data_points_ || laser_odom_queue_.size() < min_data_points_) {
            ROS_DEBUG_THROTTLE(2.0, "Not enough data: GPS=%zu, Laser=%zu", 
                              gps_odom_queue_.size(), laser_odom_queue_.size());
            return;
        }
        
        // 收集时间对齐的位姿对
        std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> aligned_poses;
        
        for (const auto& laser_odom : laser_odom_queue_) {
            auto gps_iter = findClosestGpsOdom(laser_odom.first);
            if (gps_iter != gps_odom_queue_.end()) {
                double time_diff = std::abs((laser_odom.first - gps_iter->first).toSec());
                if (time_diff < max_time_diff_) {
                    aligned_poses.emplace_back(gps_iter->second,laser_odom.second);
                }
            }
        }
        
        if (aligned_poses.size() < min_data_points_) {
            ROS_WARN_THROTTLE(2.0, "Not enough aligned poses: %zu", aligned_poses.size());
            return;
        }
        
        ROS_INFO("Computing transform using %zu aligned pose pairs", aligned_poses.size());
        
        if (computeTransformLeastSquares(aligned_poses, gps_to_laser_transform_)) 
        {
            transform_computed_ = true;
            
            // 输出变换信息
            Eigen::Vector3d translation = gps_to_laser_transform_.translation();
            Eigen::Quaterniond rotation(gps_to_laser_transform_.linear());
            
            ROS_INFO("=== GPS to Laser Transform Computed ===");
            ROS_INFO("Translation: [%.3f, %.3f, %.3f] m", 
                    translation.x(), translation.y(), translation.z());
            ROS_INFO("Rotation: [%.3f, %.3f, %.3f, %.3f]", 
                    rotation.x(), rotation.y(), rotation.z(), rotation.w());
            ROS_INFO("======================================");
            
            // 发布 TF 变换
            publishTransform();
        }
    }
    
    std::deque<std::pair<ros::Time, Eigen::Isometry3d>>::const_iterator 
    findClosestGpsOdom(const ros::Time& timestamp) {
        if (gps_odom_queue_.empty()) {
            return gps_odom_queue_.end();
        }
        
        auto closest = gps_odom_queue_.begin();
        double min_time_diff = std::abs((timestamp - closest->first).toSec());
        
        for (auto it = gps_odom_queue_.begin(); it != gps_odom_queue_.end(); ++it) {
            double time_diff = std::abs((timestamp - it->first).toSec());
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest = it;
            }
        }
        
        return closest;
    }
    
    bool computeTransformLeastSquares(const std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>>& poses,
                                     Eigen::Isometry3d& transform) {
        if (poses.empty()) {
            return false;
        }
        
        // 计算质心
        Eigen::Vector3d gps_centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d laser_centroid = Eigen::Vector3d::Zero();
        
        for (const auto& pose_pair : poses) {
            gps_centroid += pose_pair.first.translation();
            laser_centroid += pose_pair.second.translation();
        }
        
        gps_centroid /= poses.size();
        laser_centroid /= poses.size();
        
        // 计算协方差矩阵
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (const auto& pose_pair : poses) {
            Eigen::Vector3d gps_vec = pose_pair.first.translation() - gps_centroid;
            Eigen::Vector3d laser_vec = pose_pair.second.translation() - laser_centroid;
            H += laser_vec * gps_vec.transpose();
        }
        
        // SVD 分解求解最优旋转
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        
        Eigen::Matrix3d R = V * U.transpose();
        
        // 确保右手坐标系
        if (R.determinant() < 0) {
            V.col(2) *= -1;
            R = V * U.transpose();
        }
        
        // 计算平移
        Eigen::Vector3d t = laser_centroid - R * gps_centroid;
        
        // 构建变换矩阵
        transform.setIdentity();
        transform.translate(t);
        transform.rotate(R);
        
        // 计算平均误差
        double avg_error = computeTransformationError(poses, transform);
        ROS_INFO("Transformation average error: %.6f meters", avg_error);
        
        return avg_error < 0.5; // 误差阈值
    }
    
    double computeTransformationError(const std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>>& poses,
                                     const Eigen::Isometry3d& transform) {
        double total_error = 0.0;
        for (const auto& pose_pair : poses) {
            Eigen::Isometry3d transformed_gps = transform * pose_pair.first;
            // Eigen::Isometry3d transformed_gps =  pose_pair.first*transform;
            Eigen::Vector3d error = transformed_gps.translation() - pose_pair.second.translation();
            total_error += error.norm();
        }
        return total_error / poses.size();
    }
    
    void publishTransformedGpsOdometry(const nav_msgs::Odometry& gps_odom) {
        // 转换位姿
        Eigen::Isometry3d gps_pose = odometryToIsometry(gps_odom);
        Eigen::Isometry3d transformed_pose = gps_to_laser_transform_ * gps_pose;
        // Eigen::Isometry3d transformed_pose = gps_pose*gps_to_laser_transform_;
        // 创建新的里程计消息
        nav_msgs::Odometry transformed_odom;
        transformed_odom.header.stamp = gps_odom.header.stamp;
        transformed_odom.header.frame_id = laser_odom_frame_;
        transformed_odom.child_frame_id = transformed_gps_frame_;
        
        // 设置变换后的位姿
        isometryToOdometry(transformed_pose, transformed_odom);
        
        // 复制速度信息
        transformed_odom.twist = gps_odom.twist;
        
        // 转换速度到新坐标系
        transformVelocity(gps_odom.twist.twist, transformed_odom.twist.twist);
        
        // 发布变换后的里程计
        transformed_gps_pub_.publish(transformed_odom);
        
        ROS_DEBUG_THROTTLE(5.0, "Published transformed GPS odometry in laser frame");

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = transformed_gps_frame_;
        pose_stamped.pose.position.x = transformed_odom.pose.pose.position.x;
        pose_stamped.pose.position.y = transformed_odom.pose.pose.position.y;
        pose_stamped.pose.position.z = transformed_odom.pose.pose.position.z;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

        transformed_gps_path.poses.push_back(pose_stamped);
        transformed_gps_path.header.stamp = ros::Time::now();
        transformed_gps_path.header.frame_id = transformed_gps_frame_;
        transformed_gps_path_pub_.publish(transformed_gps_path);
    }
    
    void publishOriginalLaserOdometry(const nav_msgs::Odometry& laser_odom) {
        // 重新发布原始激光里程计（可选：可以修改坐标系或添加标识）
        nav_msgs::Odometry published_odom = laser_odom;
        published_odom.header.frame_id = laser_odom_frame_;
        published_odom.child_frame_id = base_frame_;
        
        original_laser_pub_.publish(published_odom);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = laser_odom_frame_;
        pose_stamped.pose.position.x = laser_odom.pose.pose.position.x;
        pose_stamped.pose.position.y = laser_odom.pose.pose.position.y;
        pose_stamped.pose.position.z = laser_odom.pose.pose.position.z;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

        original_laser_path.poses.push_back(pose_stamped);
        original_laser_path.header.stamp = ros::Time::now();
        original_laser_path.header.frame_id = laser_odom_frame_;
        original_laser_path_pub_.publish(original_laser_path);
    }
    
    void publishOriginalGpsOdometry(const nav_msgs::Odometry& gps_odom) {
        // 重新发布原始 GPS 里程计
        nav_msgs::Odometry published_odom = gps_odom;
        published_odom.header.frame_id = gps_odom_frame_;
        published_odom.child_frame_id = base_frame_;
        
        original_gps_pub_.publish(published_odom);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = gps_odom_frame_;
        pose_stamped.pose.position.x = gps_odom.pose.pose.position.x;
        pose_stamped.pose.position.y = gps_odom.pose.pose.position.y;
        pose_stamped.pose.position.z = gps_odom.pose.pose.position.z;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

        original_gps_path.poses.push_back(pose_stamped);
        original_gps_path.header.stamp = ros::Time::now();
        original_gps_path.header.frame_id = gps_odom_frame_;
        original_gps_path_pub_.publish(original_gps_path);
    }
    
    void publishAllOdometryComparison(const nav_msgs::Odometry& current_gps_odom) {
        // 创建一个包含所有里程计信息的消息用于比较
        // 这里我们简单发布变换后的GPS里程计，但可以扩展为包含更多信息
        
        nav_msgs::Odometry comparison_odom;
        comparison_odom.header.stamp = current_gps_odom.header.stamp;
        comparison_odom.header.frame_id = laser_odom_frame_;
        comparison_odom.child_frame_id = "odom_comparison";
        
        // 使用变换后的位姿
        Eigen::Isometry3d gps_pose = odometryToIsometry(current_gps_odom);
        Eigen::Isometry3d transformed_pose = gps_to_laser_transform_ * gps_pose;
        isometryToOdometry(transformed_pose, comparison_odom);
        
        // 在协方差字段中存储额外信息（可选）
        // 例如，可以在协方差中存储原始GPS位置或误差信息
        
        all_odom_pub_.publish(comparison_odom);
    }
    
    void publishTransform() {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = ros::Time::now();
        transform_msg.header.frame_id = laser_odom_frame_;
        transform_msg.child_frame_id = gps_odom_frame_;
        
        transform_msg.transform.translation.x = gps_to_laser_transform_.translation().x();
        transform_msg.transform.translation.y = gps_to_laser_transform_.translation().y();
        transform_msg.transform.translation.z = gps_to_laser_transform_.translation().z();
        
        Eigen::Quaterniond quat(gps_to_laser_transform_.linear());
        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        transform_msg.transform.rotation.w = quat.w();
        
        tf_broadcaster_->sendTransform(transform_msg);
    }
    
    void transformVelocity(const geometry_msgs::Twist& original_twist, geometry_msgs::Twist& transformed_twist) {
        // 转换线速度
        Eigen::Vector3d linear_vel(original_twist.linear.x,
                                 original_twist.linear.y,
                                 original_twist.linear.z);
        Eigen::Vector3d transformed_linear_vel = gps_to_laser_transform_.linear() * linear_vel;
        
        transformed_twist.linear.x = transformed_linear_vel.x();
        transformed_twist.linear.y = transformed_linear_vel.y();
        transformed_twist.linear.z = transformed_linear_vel.z();
        
        // 转换角速度（假设角速度在同一坐标系下不变）
        transformed_twist.angular = original_twist.angular;
    }
    
    Eigen::Isometry3d odometryToIsometry(const nav_msgs::Odometry& odom) {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        
        // 设置平移
        transform.translation().x() = odom.pose.pose.position.x;
        transform.translation().y() = odom.pose.pose.position.y;
        transform.translation().z() = odom.pose.pose.position.z;
        
        // 设置旋转
        Eigen::Quaterniond quat(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z
        );
        transform.linear() = quat.toRotationMatrix();
        
        return transform;
    }
    
    void isometryToOdometry(const Eigen::Isometry3d& transform, nav_msgs::Odometry& odom) {
        // 设置位置
        odom.pose.pose.position.x = transform.translation().x();
        odom.pose.pose.position.y = transform.translation().y();
        odom.pose.pose.position.z = transform.translation().z();
        
        // 设置方向
        Eigen::Quaterniond quat(transform.linear());
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        
        // 设置协方差（如果原始数据有）
        // odom.pose.covariance = ...;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_odom_publisher");
    
    MultiOdomPublisher publisher;
    
    ROS_INFO("Multi Odom Publisher node started");
    ROS_INFO("Subscribing to:");
    ROS_INFO("  - GPS Odometry: /gps_odom");
    ROS_INFO("  - Laser Odometry: /laser_odom");
    ROS_INFO("Publishing multiple odometry topics for comparison");
    
    ros::spin();
    
    return 0;
}