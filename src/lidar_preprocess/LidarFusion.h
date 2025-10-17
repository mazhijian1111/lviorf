#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;
namespace RSHelios_ros
{
    // rslidar和velodyne的格式有微小的区别
    // rslidar的点云格式
    struct RsPointXYZIRT
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring = 0;
        double timestamp = 0;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(RSHelios_ros::RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))




class LidarFusion
{
public:
    LidarFusion() : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        nh.param<int>("preprocess/lidar_type", lidar_type, 0);
        nh.param<bool>("preprocess/transform_frame_id", transform_frame_id, false);
        nh.param<bool>("preprocess/CompressedImage", CompressedImage, false);
        nh.param<bool>("preprocess/lidar_back_left_helios32", lidar_back_left_helios32, false);
        nh.param<bool>("preprocess/lidar_front_right_helios32", lidar_front_right_helios32, false);


        // 初始化8个雷达的订阅和发布
        if (transform_frame_id)
        {                                                                                     // 消息转换，区分frame id
            pubs_.push_back(nh.advertise<PointCloud2>("/lidar_back_left_helios32_new", 1));   // 创建发布者
            pubs_.push_back(nh.advertise<PointCloud2>("/lidar_front_right_helios32_new", 1)); // 创建发布者


            subs_.push_back(nh.subscribe<PointCloud2>(
                "/lidar_back_left_helios32", 1,
                boost::bind(&LidarFusion::cloudCallback, this, _1, 1))); // 创建订阅者
            subs_.push_back(nh.subscribe<PointCloud2>(
                "/lidar_front_right_helios32", 1,
                boost::bind(&LidarFusion::cloudCallback, this, _1, 2))); // 创建订阅者


            sub_back_left_.subscribe(nh, "/lidar_back_left_helios32_new", 1);
            // sub_back_up_.subscribe(nh, "/lidar_back_up_helios32_new", 1);
            sub_front_right_.subscribe(nh, "/lidar_front_right_helios32_new", 1);

        }
        else
        {
            // 初始化订阅者
            sub_back_left_.subscribe(nh, "/lidar_back_left_helios32", 1);
            // sub_back_up_.subscribe(nh, "/lidar_back_up_helios32", 1);
            sub_front_right_.subscribe(nh, "/lidar_front_right_helios32", 1);

        }

        // 创建同步策略
        sync_.reset(new Synchronizer<SyncPolicy>(SyncPolicy(10),
                                                 sub_back_left_, 
                                                 sub_front_right_
                                                 ));
        sync_->registerCallback(boost::bind(&LidarFusion::fusionCallback,
                                            this, _1, _2));

        // 初始化发布者
        pub_ = nh.advertise<PointCloud2>("/fused_pointcloud", 1);
    }
    bool Is_Compressed_Image()
    {
        return this->CompressedImage;
    }

private:
    void fusionCallback(const PointCloud2ConstPtr &cloud1,const PointCloud2ConstPtr &cloud3)
    {
        try
        {
            // 合并点云
            if (lidar_type == 4)
            {
                // 转换点云到基坐标系,rslidar数据
                auto t1 = transformCloud_rslidar(cloud1, "base_link");
                auto t3 = transformCloud_rslidar(cloud3, "base_link");

                // auto t8 = transformCloud_rslidar(cloud8, "base_link");
                pcl::PointCloud<RSHelios_ros::RsPointXYZIRT> fused_cloud;
                if (lidar_back_left_helios32)
                {
                    fused_cloud += *t1;
                }
                if (lidar_front_right_helios32)
                {
                    fused_cloud += *t3;
                }

                pcl::PointCloud<RSHelios_ros::RsPointXYZIRT> filtered_cloud;
                for (const auto &point : fused_cloud.points)
                {
                    pcl::PointXYZ pt;
                    pt.x = point.x;
                    pt.y = point.y;
                    pt.z = point.z;

                    // 检查点是否有效（非NaN和非无限远）
                    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                        filtered_cloud.points.push_back(point);
                    }
                }

                fused_cloud.is_dense = true;

                // // 移除NaN点
                // std::vector<int> indices;
                // pcl::removeNaNFromPointCloud(filtered_cloud, filtered_cloud, indices);
   
                // 发布结果
                PointCloud2 output;
                pcl::toROSMsg(filtered_cloud, output);
                output.header.stamp = ros::Time().fromSec(cloud1->header.stamp.toSec());
                output.header.frame_id = "base_link";
                pub_.publish(output);
            }
            else
            {
                // 转换点云到基坐标系
                auto t1 = transformCloud(cloud1, "base_link");
                auto t3 = transformCloud(cloud3, "base_link");

                pcl::PointCloud<pcl::PointXYZI> fused_cloud;
                if (lidar_back_left_helios32)
                {
                    fused_cloud += *t1;
                }

                if (lidar_front_right_helios32)
                {
                    fused_cloud += *t3;
                }
                fused_cloud.is_dense = true;

                // 发布结果
                PointCloud2 output;
                pcl::toROSMsg(fused_cloud, output);
                output.header.stamp = cloud1->header.stamp;
                output.header.frame_id = "base_link";
                pub_.publish(output);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform failed: %s", ex.what());
        }
    }
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, const int radar_idx)
    {
        // 创建消息副本
        sensor_msgs::PointCloud2 modified_cloud = *msg;
        // 修改frame_id
        modified_cloud.header.frame_id = "lidar" + std::to_string(radar_idx);
        // 发布修改后的消息
        pubs_[radar_idx - 1].publish(modified_cloud);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformCloud(const PointCloud2ConstPtr &cloud, const std::string &target_frame)
    {

        PointCloud2 transformed_cloud;
        tf_buffer_.transform(*cloud, transformed_cloud, target_frame, ros::Duration(0.1));

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);
        return pcl_cloud;
    }
    pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr transformCloud_rslidar(const PointCloud2ConstPtr &cloud,const std::string &target_frame)
    {
        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr pcl_cloud1(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
        pcl::fromROSMsg(*cloud,*pcl_cloud1);
        PointCloud2 cloud_tmp;
        cloud_tmp.header = cloud->header;
        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr pcl_cloud2(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
        for(auto &point: pcl_cloud1->points)
        {
            double distance = sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
            if(distance > 100 || distance < 5.0)
            {
                continue;
            }
            if(pcl::isFinite(point))
            {
                pcl_cloud2->push_back(point);
                pcl_cloud2->back().ring = point.ring;
                pcl_cloud2->back().timestamp = point.timestamp;
                pcl_cloud2->back().intensity = point.intensity;
                // std::cout<<"ring="<<point.ring;
                // pcl_cloud2.ring = point.ring;
            }
        }
        // std::cout<<"input size:"<<pcl_cloud1->points.size()<<",valid size:"<<pcl_cloud2->points.size()<<std::endl;

        pcl::toROSMsg(*pcl_cloud2,cloud_tmp);
        cloud_tmp.header = cloud->header;
        PointCloud2 transformed_cloud;
        tf_buffer_.transform(cloud_tmp, transformed_cloud, target_frame, ros::Duration(0.1));

        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr pcl_cloud(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);
        return pcl_cloud;
    }
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    message_filters::Subscriber<PointCloud2> sub_back_left_;
    message_filters::Subscriber<PointCloud2> sub_front_right_;



    boost::shared_ptr<Synchronizer<SyncPolicy>> sync_;
    ros::Publisher pub_;
    std::vector<ros::Subscriber> subs_;
    std::vector<ros::Publisher> pubs_;
    int lidar_type;
    bool transform_frame_id;
    bool CompressedImage;
    bool lidar_back_left_helios32;
    bool lidar_front_right_helios32;
};
