#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <mutex>
#include <queue>
#include <deque>
#include <thread>
#include <atomic>
#include <condition_variable>

struct OdometryData {
    ros::Time stamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Matrix<double, 6, 6> covariance;
    std::string sensor_type;
    std::string frame_id;
    
    OdometryData() : stamp(0) {}
};

class MultiSensorFusion {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber visual_odom_sub_;
    ros::Subscriber laser_odom_sub_;
    ros::Subscriber gnss_odom_sub_;
    
    // 发布器
    ros::Publisher fused_odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // TF2相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 数据缓冲区
    std::deque<OdometryData> visual_odom_buffer_;
    std::deque<OdometryData> laser_odom_buffer_;
    std::deque<OdometryData> gnss_odom_buffer_;
    
    std::mutex data_mutex_;
    std::mutex tf_mutex_;
    
    // 多线程相关
    std::thread optimization_thread_;
    std::thread publishing_thread_;
    std::atomic<bool> running_{false};
    std::condition_variable data_cv_;
    
    // 参数
    double max_buffer_time_;
    double optimization_rate_;
    double publishing_rate_;
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string world_frame_id_;
    
    // 传感器到基座的变换
    std::map<std::string, Eigen::Isometry3d> sensor_transforms_;
    
    // 当前最优估计
    Eigen::Vector3d current_position_;
    Eigen::Quaterniond current_orientation_;
    ros::Time last_optimization_time_;

public:
    MultiSensorFusion() : 
        private_nh_("~"),
        tf_listener_(tf_buffer_)
    {
        // 参数初始化
        private_nh_.param("max_buffer_time", max_buffer_time_, 2.0);
        private_nh_.param("optimization_rate", optimization_rate_, 10.0);
        private_nh_.param("publishing_rate", publishing_rate_, 50.0);
        private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
        private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
        private_nh_.param("world_frame_id", world_frame_id_, std::string("world"));
        
        // 初始化传感器变换（默认单位变换）
        sensor_transforms_["visual"] = Eigen::Isometry3d::Identity();
        sensor_transforms_["laser"] = Eigen::Isometry3d::Identity();
        sensor_transforms_["gnss"] = Eigen::Isometry3d::Identity();
        
        // 初始化状态
        current_position_ = Eigen::Vector3d::Zero();
        current_orientation_ = Eigen::Quaterniond::Identity();
        last_optimization_time_ = ros::Time::now();
        
        // 订阅话题
        visual_odom_sub_ = nh_.subscribe("visual_odom", 10, 
                                        &MultiSensorFusion::visualOdomCallback, this);
        laser_odom_sub_ = nh_.subscribe("laser_odom", 10, 
                                       &MultiSensorFusion::laserOdomCallback, this);
        gnss_odom_sub_ = nh_.subscribe("gnss_odom", 10, 
                                      &MultiSensorFusion::gnssOdomCallback, this);
        
        // 发布话题
        fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("fused_odom", 10);
        
        // 启动优化线程和发布线程
        running_ = true;
        optimization_thread_ = std::thread(&MultiSensorFusion::optimizationLoop, this);
        publishing_thread_ = std::thread(&MultiSensorFusion::publishingLoop, this);
        
        ROS_INFO("Multi-sensor fusion node initialized with 2 threads");
    }
    
    ~MultiSensorFusion() {
        running_ = false;
        data_cv_.notify_all();
        
        if (optimization_thread_.joinable()) {
            optimization_thread_.join();
        }
        if (publishing_thread_.joinable()) {
            publishing_thread_.join();
        }
    }
    
    void visualOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        processOdometryData(msg, "visual");
    }
    
    void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        processOdometryData(msg, "laser");
    }
    
    void gnssOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        processOdometryData(msg, "gnss");
    }
    
    void processOdometryData(const nav_msgs::Odometry::ConstPtr& msg, const std::string& sensor_type) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        OdometryData data;
        data.stamp = msg->header.stamp;
        data.sensor_type = sensor_type;
        data.frame_id = msg->header.frame_id;
        
        try {
            // 转换到世界坐标系
            geometry_msgs::PoseStamped input_pose, transformed_pose;
            input_pose.header = msg->header;
            input_pose.pose = msg->pose.pose;
            
            // 使用TF2进行坐标变换
            tf_buffer_.transform(input_pose, transformed_pose, world_frame_id_);
            
            // 提取位置
            data.position << transformed_pose.pose.position.x,
                            transformed_pose.pose.position.y,
                            transformed_pose.pose.position.z;
            
            // 提取方向
            data.orientation = Eigen::Quaterniond(
                transformed_pose.pose.orientation.w,
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z
            );
            
            // 提取协方差矩阵
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    data.covariance(i, j) = msg->pose.covariance[i * 6 + j];
                }
            }
            
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF transformation failed: %s", ex.what());
            return;
        }
        
        // 添加到相应的缓冲区
        if (sensor_type == "visual") {
            visual_odom_buffer_.push_back(data);
            cleanupBuffer(visual_odom_buffer_);
        } else if (sensor_type == "laser") {
            laser_odom_buffer_.push_back(data);
            cleanupBuffer(laser_odom_buffer_);
        } else if (sensor_type == "gnss") {
            gnss_odom_buffer_.push_back(data);
            cleanupBuffer(gnss_odom_buffer_);
        }
        
        // 通知优化线程有新数据
        data_cv_.notify_one();
    }
    
    void cleanupBuffer(std::deque<OdometryData>& buffer) {
        ros::Time now = ros::Time::now();
        while (!buffer.empty() && 
               (now - buffer.front().stamp).toSec() > max_buffer_time_) {
            buffer.pop_front();
        }
    }
    
    void optimizationLoop() {
        ros::Rate rate(optimization_rate_);
        
        while (running_ && ros::ok()) {
            std::unique_lock<std::mutex> lock(data_mutex_);
            
            // 等待新数据或超时
            data_cv_.wait_for(lock, std::chrono::milliseconds(100));
            
            if (!hasData()) {
                continue;
            }
            
            // 执行优化
            performOptimization();
            
            lock.unlock();
            rate.sleep();
        }
    }
    
    void publishingLoop() {
        ros::Rate rate(publishing_rate_);
        
        while (running_ && ros::ok()) {
            // 发布优化结果
            publishFusedOdometry();
            rate.sleep();
        }
    }
    
    bool hasData() const {
        return !visual_odom_buffer_.empty() || 
               !laser_odom_buffer_.empty() || 
               !gnss_odom_buffer_.empty();
    }
    
    void performOptimization() {
        if (!hasData()) {
            return;
        }
        
        // 创建Ceres问题
        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        
        // 配置优化选项
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 50;
        options.minimizer_progress_to_stdout = false;
        options.function_tolerance = 1e-6;
        options.parameter_tolerance = 1e-8;
        
        // 优化变量
        double position[3] = {current_position_.x(), current_position_.y(), current_position_.z()};
        double orientation[4] = {current_orientation_.w(), current_orientation_.x(), 
                                current_orientation_.y(), current_orientation_.z()};
        
        // 添加参数块
        problem.AddParameterBlock(position, 3);
        problem.AddParameterBlock(orientation, 4, new ceres::QuaternionParameterization());
        
        // 添加残差项
        addSensorResiduals(problem, visual_odom_buffer_, "visual", 1.0);
        addSensorResiduals(problem, laser_odom_buffer_, "laser", 0.8);
        addSensorResiduals(problem, gnss_odom_buffer_, "gnss", 0.3);
        
        // 运行优化
        ceres::Solve(options, &problem, &summary);
        
        // 更新最优估计
        current_position_ = Eigen::Vector3d(position[0], position[1], position[2]);
        current_orientation_ = Eigen::Quaterniond(orientation[0], orientation[1], 
                                                 orientation[2], orientation[3]);
        
        last_optimization_time_ = ros::Time::now();
    }
    
    void addSensorResiduals(ceres::Problem& problem, 
                           const std::deque<OdometryData>& buffer, 
                           const std::string& sensor_type,
                           double weight) {
        if (buffer.empty()) return;
        
        // 使用最新的数据
        const OdometryData& latest_data = buffer.back();
        
        // 位置残差
        ceres::CostFunction* position_cost_function = 
            new ceres::AutoDiffCostFunction<PositionResidual, 3, 3>(
                new PositionResidual(latest_data.position, weight));
        problem.AddResidualBlock(position_cost_function, nullptr, current_position_.data());
        
        // 方向残差
        ceres::CostFunction* orientation_cost_function = 
            new ceres::AutoDiffCostFunction<OrientationResidual, 3, 4>(
                new OrientationResidual(latest_data.orientation, weight));
        problem.AddResidualBlock(orientation_cost_function, nullptr, current_orientation_.coeffs().data());
    }
    
    void publishFusedOdometry() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        
        // 设置位置
        odom_msg.pose.pose.position.x = current_position_.x();
        odom_msg.pose.pose.position.y = current_position_.y();
        odom_msg.pose.pose.position.z = current_position_.z();
        
        // 设置方向
        odom_msg.pose.pose.orientation.w = current_orientation_.w();
        odom_msg.pose.pose.orientation.x = current_orientation_.x();
        odom_msg.pose.pose.orientation.y = current_orientation_.y();
        odom_msg.pose.pose.orientation.z = current_orientation_.z();
        
        // 设置协方差
        setCovarianceMatrix(odom_msg.pose.covariance);
        
        // 发布里程计
        fused_odom_pub_.publish(odom_msg);
        
        // 发布TF
        publishTF(odom_msg);
    }
    
    void setCovarianceMatrix(boost::array<double, 36>& covariance) {
        // 基于传感器数据质量设置协方差
        std::fill(covariance.begin(), covariance.end(), 0.0);
        
        double pos_cov = 0.01;
        double rot_cov = 0.02;
        
        // 根据可用传感器调整协方差
        if (!visual_odom_buffer_.empty()) {
            pos_cov *= 0.8;
            rot_cov *= 0.5;
        }
        if (!laser_odom_buffer_.empty()) {
            pos_cov *= 0.6;
        }
        if (!gnss_odom_buffer_.empty()) {
            pos_cov *= 0.3;
            rot_cov *= 1.5;  // GNSS对方向估计帮助不大
        }
        
        covariance[0] = pos_cov;   // x
        covariance[7] = pos_cov;   // y
        covariance[14] = pos_cov;  // z
        covariance[21] = rot_cov;  // roll
        covariance[28] = rot_cov;  // pitch
        covariance[35] = rot_cov;  // yaw
    }
    
    void publishTF(const nav_msgs::Odometry& odom_msg) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ));
        
        tf::Quaternion q(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        );
        transform.setRotation(q);
        
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, odom_msg.header.stamp, 
                               odom_frame_id_, base_frame_id_));
    }
    
    // 残差函数定义
    struct PositionResidual {
        PositionResidual(const Eigen::Vector3d& measured_pos, double weight)
            : measured_pos_(measured_pos), weight_(weight) {}
        
        template<typename T>
        bool operator()(const T* const estimated_pos, T* residual) const {
            residual[0] = weight_ * (estimated_pos[0] - T(measured_pos_.x()));
            residual[1] = weight_ * (estimated_pos[1] - T(measured_pos_.y()));
            residual[2] = weight_ * (estimated_pos[2] - T(measured_pos_.z()));
            return true;
        }
        
    private:
        Eigen::Vector3d measured_pos_;
        double weight_;
    };
    
    struct OrientationResidual {
        OrientationResidual(const Eigen::Quaterniond& measured_quat, double weight)
            : measured_quat_(measured_quat), weight_(weight) {}
        
        template<typename T>
        bool operator()(const T* const estimated_quat, T* residual) const {
            T measured_quat[4] = {T(measured_quat_.w()), T(measured_quat_.x()), 
                                 T(measured_quat_.y()), T(measured_quat_.z())};
            
            T relative_quat[4];
            ceres::QuaternionProduct(measured_quat, estimated_quat, relative_quat);
            
            residual[0] = weight_ * relative_quat[1];
            residual[1] = weight_ * relative_quat[2];
            residual[2] = weight_ * relative_quat[3];
            
            return true;
        }
        
    private:
        Eigen::Quaterniond measured_quat_;
        double weight_;
    };
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_sensor_fusion_node");
    ROS_INFO("\033[1;32m----> multi_sensor_fusion_node Started.\033[0m");
    
    MultiSensorFusion fusion_node;
    
    ros::spin();
    
    return 0;
}