#include "utility.h"
#include "lviorf/cloud_info.h"
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


// <!-- lviorf_localization_yjz_lucky_boy -->
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring) (std::uint16_t, noise, noise) (std::uint32_t, range, range)
)

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT, 
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (std::uint16_t, ring, ring)(double, timestamp, timestamp)
)

// mulran datasets
struct MulranPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 }EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT (MulranPointXYZIRT,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (std::uint32_t, t, t) (int, ring, ring)
 )






// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

using namespace sensor_msgs;
using namespace message_filters;
typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;


class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;
    std::mutex odoVIOLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom, subVIOOdom;
    std::deque<nav_msgs::Odometry> odomQueue;
    std::deque<nav_msgs::Odometry> odomVIOQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    double last_angular_x, last_angular_y, last_angular_z;//上一帧的角速度

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;

    int deskewFlag;

    bool odomDeskewFlag, odomVIODeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lviorf::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    message_filters::Subscriber<PointCloud2> sub_back_left_;
    message_filters::Subscriber<PointCloud2> sub_front_right_;
    boost::shared_ptr<Synchronizer<SyncPolicy>> sync_;

    double last_imu_time = -1;
    bool first_imu = true;

public:
    //构造函数
    ImageProjection():deskewFlag(0),tf_listener_(tf_buffer_)
    {
        //订阅IMU原始数据
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        //订阅激光里程计增量
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        //订阅视觉里程计
        subVIOOdom       = nh.subscribe<nav_msgs::Odometry> ("lviorf/vins/odometry/imu_propagate_ros", 2000, &ImageProjection::odometryVIOHandler, this, ros::TransportHints().tcpNoDelay());
        
        //订阅输入的激光雷达话题
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        //发布去畸变后的点云
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lviorf/lidar/deskew/cloud_deskewed", 1); //去除畸变后的点云
        
        //发布自定义格式的点云（带IMU预积分、里程计信息）
        pubLaserCloudInfo = nh.advertise<lviorf::cloud_info> ("lviorf/deskew/cloud_info", 1); //自定义格式的点云，含有更多的信息

        // 初始化订阅者
        sub_back_left_.subscribe(nh, "/lidar_back_left_helios32", 1);
        sub_front_right_.subscribe(nh, "/lidar_front_right_helios32", 1);

                // 创建同步策略
        sync_.reset(new Synchronizer<SyncPolicy>(SyncPolicy(10),
                                                 sub_back_left_, 
                                                 sub_front_right_
                                                 ));
        sync_->registerCallback(boost::bind(&ImageProjection::fusionCallback,
                                            this, _1, _2));

        pub_ = nh.advertise<PointCloud2>("/fused_points", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        fullCloud->clear();

        imuPointerCur = 0; //IMU指针
        firstPointFlag = true;
        odomDeskewFlag = false;
        odomVIODeskewFlag = false;//视觉里程计去畸变标志

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

    }

    ~ImageProjection(){}

    void fusionCallback(const PointCloud2ConstPtr &cloud1,const PointCloud2ConstPtr &cloud3)
    {
        try
        {
                sensor_msgs::PointCloud2 cloud11 = *cloud1;
                sensor_msgs::PointCloud2 cloud33 = *cloud3;
                cloud11.header.frame_id = "lidar1";
                cloud33.header.frame_id = "lidar2";
                // 转换点云到基坐标系,rslidar数据
                auto t1 = transformCloud_rslidar(cloud11, "base_link");
                auto t3 = transformCloud_rslidar(cloud33, "base_link");

                // auto t8 = transformCloud_rslidar(cloud8, "base_link");
                pcl::PointCloud<RSHelios_ros::RsPointXYZIRT> fused_cloud;
                // if (lidar_back_left_helios32)
                // {
                    fused_cloud += *t1;
                // }
                // if (lidar_front_right_helios32)
                // {
                    fused_cloud += *t3;
                // }

                pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr filtered_cloud(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
                for (const auto &point : fused_cloud.points)
                {
                    RSHelios_ros::RsPointXYZIRT pt;
                    pt.x = point.x;
                    pt.y = point.y;
                    pt.z = point.z;
                    pt.intensity = point.intensity;
                    pt.ring = point.ring;
                    pt.timestamp = point.timestamp;
                    // std::cout<<"ring:"<<pt.ring<<",pt.timestamp="<<pt.timestamp<<std::endl;


                    // 检查点是否有效（非NaN和非无限远）
                    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                        filtered_cloud->points.push_back(pt);
                    }
                }

                filtered_cloud->is_dense = true;

                // // 2. downsample new cloud (save memory)
                // pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr laser_cloud_in_ds(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>());
                // static pcl::VoxelGrid<RSHelios_ros::RsPointXYZIRT> downSizeFilter;
                // downSizeFilter.setLeafSize(0.2, 0.2, 0.1);
                // downSizeFilter.setInputCloud(filtered_cloud); //
                // downSizeFilter.filter(*laser_cloud_in_ds);
                // std::cout<<"filter before:"<<filtered_cloud->points.size()<<",after:"<<laser_cloud_in_ds->points.size()<<std::endl;
                // *filtered_cloud = *laser_cloud_in_ds;
                

                std::sort(filtered_cloud->points.begin(), filtered_cloud->points.end(),
                [](const RSHelios_ros::RsPointXYZIRT& a, const RSHelios_ros::RsPointXYZIRT& b) {
                    return a.timestamp < b.timestamp;
                });

                double min_time = filtered_cloud->points.front().timestamp;


                // 发布结果
                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(*filtered_cloud, output);
                output.header.stamp = ros::Time().fromSec(min_time);
                output.header.frame_id = "base_link";
                //转到前雷达下
                sensor_msgs::PointCloud2 transformed_cloud;
                tf_buffer_.transform(output, transformed_cloud, "lidar2", ros::Duration(0.1));
                // transformed_cloud.header.stamp = ros::Time().fromSec(min_time);
                // transformed_cloud.header.frame_id = "lidar2";
                pub_.publish(transformed_cloud);

                FusedCloudHandle(transformed_cloud);



            
       
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform failed: %s", ex.what());
        }
    }

    pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr transformCloud_rslidar(PointCloud2 &cloud,const std::string &target_frame)
    {
        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr pcl_cloud1(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
        pcl::fromROSMsg(cloud,*pcl_cloud1);
        PointCloud2 cloud_tmp;
        cloud_tmp.header = cloud.header;
        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr pcl_cloud2(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
        for(auto &point: pcl_cloud1->points)
        {
            double distance = sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
            if(distance > 200 || distance < 5.0)
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
        cloud_tmp.header = cloud.header;
        PointCloud2 transformed_cloud;
        tf_buffer_.transform(cloud_tmp, transformed_cloud, target_frame, ros::Duration(0.1));

        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr pcl_cloud(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);
        return pcl_cloud;
    }


    //IMU回调函数，订阅到imu数据帧后存入队列中
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        // ROS_INFO("imuHandler");
        if(first_imu)
        {
            first_imu = false;
            last_imu_time = imuMsg->header.stamp.toSec();
        }

        if(abs(imuMsg->header.stamp.toSec()-last_imu_time) > 15.0/imuRate)
        {
            last_imu_time = imuMsg->header.stamp.toSec();
            // resetParams(); //重置标志位（系统初始化标志位、第一次优化标志位、第一次优化时间）
            return;
        }


        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        //使用厦门数据时
        // sensor_msgs::Imu thisImu = imuConverterInXiamen(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
        last_imu_time = imuMsg->header.stamp.toSec();
    }


    //激光里程计回调函数,存入到odomQueue队列中
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    //VIO里程计回调函数，存入到odomVIOQueue队列中
    void odometryVIOHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock3(odoVIOLock);
        odomVIOQueue.push_back(*odometryMsg);
    }

    //激光点云回调函数，对激光点云进行去畸变处理
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        ROS_INFO("Input lidar cloudHandler");
        //检查激光点云是否符合要求
        if (!cachePointCloud(laserCloudMsg))
            return;

        //把IMU、激光里程计、视觉里程计的信息赋值到自定义的激光帧上
        if (!deskewInfo())
            return;

        //真正的点云去畸变处理
        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    void FusedCloudHandle(sensor_msgs::PointCloud2 laserCloudMsg)
    {
        ROS_INFO("FusedCloudHandle start");
        //检查激光点云是否符合要求
        if (!cacheFusePointCloud(laserCloudMsg))
        {
            std::cout<<"当前帧激光点云不符合要求,return"<<std::endl;
            return;
        }

        //把IMU、激光里程计、视觉里程计的信息赋值到自定义的激光帧上
        if (!deskewInfo())
        {
            std::cout<<"当前帧激光点云不符合要求,无IMU信息,return"<<std::endl;
            return;
        }    

        //真正的点云去畸变处理
        projectPointCloud();

        ROS_INFO("FusedCloudHandle end");

        publishClouds();

        resetParameters();
    }


    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {

        // cache point cloud，前面两帧点云都不处理
        
        cloudQueue.push_back(*laserCloudMsg);
        std::cout<<"订阅到的输入点云帧队列大小："<<cloudQueue.size()<<std::endl;
        if (cloudQueue.size() <= 2) //点云队列中的点云帧数量小于2，直接返回
            return false;

        //从队列中取出最早的点云帧
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT> pl_orig_RSHelios;
        // pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>::Ptr pl_orig_RSHelios1(new pcl::PointCloud<RSHelios_ros::RsPointXYZIRT>);
        // pcl::fromROSMsg(currentCloudMsg, *pl_orig_RSHelios1);
        // for (const RSHelios_ros::RsPointXYZIRT point : pl_orig_RSHelios1->points)
        // {
        //     // std::cout<<"ring:"<<point.ring<<",timestamp="<<point.timestamp<<std::endl;
        // }
        
        //根据激光雷达的类型处理点云数据
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        } // <!-- lviorf_yjz_lucky_boy -->
        else if (sensor == SensorType::MULRAN)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
            {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = static_cast<float>(src.t);
            }
        } // <!-- lviorf_yjz_lucky_boy -->
        else if (sensor == SensorType::ROBOSENSE) {
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            // Convert to robosense format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;

            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        }  //<!-- mzj -->
        else if(sensor == SensorType::RSHelios)
        {
            // Convert to Velodyne format
            std::cout<<"RSHelios 雷达"<<std::endl;
            pcl::fromROSMsg(currentCloudMsg, pl_orig_RSHelios);

            pcl::PointCloud<RSHelios_ros::RsPointXYZIRT> filtered_cloud;
            for (const auto &point : pl_orig_RSHelios.points)
            {
                // std::cout<<"ring:"<<point.ring<<",pt.timestamp="<<point.timestamp<<std::endl;

                double distance = sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
                if(distance > 100 || distance < 5.0)
                {
                    continue;
                }

                // 检查点是否有效（非NaN和非无限远）
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    filtered_cloud.push_back(point);
                }
            }
            filtered_cloud.is_dense = true;
            // std::cout<<"nan前:"<<pl_orig_RSHelios.size()<<",nan后:"<<filtered_cloud.size()<<std::endl;

            laserCloudIn->points.resize(filtered_cloud.size());
            laserCloudIn->is_dense = filtered_cloud.is_dense;
            // to new pointcloud
            double start_stamptime = filtered_cloud.points[0].timestamp;
            for (int i = 0; i < filtered_cloud.points.size(); i++) {
                auto &src = filtered_cloud.points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
                // ROS_INFO("%.11f,%.6f",src.timestamp,dst.time);
                // std::cout<<src.ring<<",dst.ring="<<dst.ring<<std::endl;
                // std::cout<<src.timestamp<<",dst.time="<<dst.time<<std::endl;
                // if (has_nan(pl_orig_RSHelios.points[point_id]))
                //     continue;
                // velodyne_ros::Point new_point;
                // new_point.x = pl_orig_RSHelios.points[point_id].x;
                // new_point.y = pl_orig_RSHelios.points[point_id].y;
                // new_point.z = pl_orig_RSHelios.points[point_id].z;
                // new_point.intensity = pl_orig_RSHelios.points[point_id].intensity;
                // new_point.ring = pl_orig_RSHelios.points[point_id].ring;
                // // 计算相对于第一个点的相对时间
                // // new_point.time = pl_orig_RSHelios.points[point_id].timestamp;
                // new_point.time =float(pl_orig_RSHelios.points[point_id].timestamp -
                //                     pl_orig_RSHelios.points[0].timestamp);
                // if(point_id <12)
                // {
                // printf("point_id:%d (%f, %f, %f, %f) ring:%d timestamp:%lf timestamp0:%lf new_point.time:%f\n",point_id,new_point.x,new_point.y,new_point.z,
                // new_point.intensity,new_point.ring,pl_orig_RSHelios.points[point_id].timestamp,pl_orig_RSHelios.points[0].timestamp,new_point.time);
                // }
                // pl_orig.points.push_back(new_point);
            }
            
        } 
        else {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        //点云校验，非RSHelios32 雷达
        if(sensor != SensorType::RSHelios)
        {
            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
            // std::cout<<"当前点云帧开始时间："<<timeScanCur<<",持续时间:"<<laserCloudIn->points.back().time<<std::endl;

            // check dense flag
            if (laserCloudIn->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }

            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }

            // check point time
            //点的时间校验
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (auto &field : currentCloudMsg.fields)
                {
                    if (field.name == "time" || field.name == "t")
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1) //点无时间戳信息
                {
                    std::cout<<"点云中的点无时间信息"<<std::endl;
                    // ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
                    
                }
            }

        }
        else //针对RSHelios 雷达点云的校验
        {
            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
            // std::cout<<"当前点云帧开始时间："<<timeScanCur<<",持续时间:"<<laserCloudIn->points.back().time<<std::endl;

            // check dense flag
            if (laserCloudIn->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }

            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        std::cout<<"点云中有线数信息"<<std::endl;
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    std::cout<<"点云中无线数信息"<<std::endl;
                    ros::shutdown();
                }
            }

            // check point time
            //点的时间校验
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (auto &field : pl_orig_RSHelios.points)
                {
                    if (abs(field.timestamp - pl_orig_RSHelios.points[0].timestamp) >= 1e-6)
                    {
                        std::cout<<"点云中的点有时间信息"<<std::endl;
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1) //点无时间戳信息
                {
                    std::cout<<"点云中的点无时间信息"<<std::endl;
                    // ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
                    
                }
            }
        }

        return true;
    }

    bool cacheFusePointCloud(sensor_msgs::PointCloud2 laserCloudMsg)
    {

        // cache point cloud，前面两帧点云都不处理
        
        cloudQueue.push_back(laserCloudMsg);
        // std::cout<<"订阅到的输入点云帧队列大小："<<cloudQueue.size()<<std::endl;
        if (cloudQueue.size() <= 2) //点云队列中的点云帧数量小于2，直接返回
            return false;

        //从队列中取出最早的点云帧
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        pcl::PointCloud<RSHelios_ros::RsPointXYZIRT> pl_orig_RSHelios;
        
        //根据激光雷达的类型处理点云数据
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        } // <!-- lviorf_yjz_lucky_boy -->
        else if (sensor == SensorType::MULRAN)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
            {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = static_cast<float>(src.t);
            }
        } // <!-- lviorf_yjz_lucky_boy -->
        else if (sensor == SensorType::ROBOSENSE) {
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            // Convert to robosense format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;

            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        }  //<!-- mzj -->
        else if(sensor == SensorType::RSHelios)
        {
            // Convert to Velodyne format
            // std::cout<<"RSHelios 雷达"<<std::endl;
            pcl::fromROSMsg(currentCloudMsg, pl_orig_RSHelios);

            pcl::PointCloud<RSHelios_ros::RsPointXYZIRT> filtered_cloud;
            for (const auto &point : pl_orig_RSHelios.points)
            {
                // std::cout<<"ring:"<<point.ring<<",pt.timestamp="<<point.timestamp<<std::endl;

                double distance = sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
                if(distance > 100 || distance < 5.0)
                {
                    continue;
                }

                // 检查点是否有效（非NaN和非无限远）
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    filtered_cloud.push_back(point);
                }
            }
            filtered_cloud.is_dense = true;
            // std::cout<<"nan前:"<<pl_orig_RSHelios.size()<<",nan后:"<<filtered_cloud.size()<<std::endl;

            laserCloudIn->points.resize(filtered_cloud.size());
            laserCloudIn->is_dense = filtered_cloud.is_dense;
            // to new pointcloud
            double start_stamptime = filtered_cloud.points[0].timestamp;
            for (int i = 0; i < filtered_cloud.points.size(); i++) {
                auto &src = filtered_cloud.points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
                // ROS_INFO("%.11f,%.6f",src.timestamp,dst.time);
                // std::cout<<src.ring<<",dst.ring="<<dst.ring<<std::endl;
                // std::cout<<src.timestamp<<",dst.time="<<dst.time<<std::endl;
                // if (has_nan(pl_orig_RSHelios.points[point_id]))
                //     continue;
                // velodyne_ros::Point new_point;
                // new_point.x = pl_orig_RSHelios.points[point_id].x;
                // new_point.y = pl_orig_RSHelios.points[point_id].y;
                // new_point.z = pl_orig_RSHelios.points[point_id].z;
                // new_point.intensity = pl_orig_RSHelios.points[point_id].intensity;
                // new_point.ring = pl_orig_RSHelios.points[point_id].ring;
                // // 计算相对于第一个点的相对时间
                // // new_point.time = pl_orig_RSHelios.points[point_id].timestamp;
                // new_point.time =float(pl_orig_RSHelios.points[point_id].timestamp -
                //                     pl_orig_RSHelios.points[0].timestamp);
                // if(point_id <12)
                // {
                // printf("point_id:%d (%f, %f, %f, %f) ring:%d timestamp:%lf timestamp0:%lf new_point.time:%f\n",point_id,new_point.x,new_point.y,new_point.z,
                // new_point.intensity,new_point.ring,pl_orig_RSHelios.points[point_id].timestamp,pl_orig_RSHelios.points[0].timestamp,new_point.time);
                // }
                // pl_orig.points.push_back(new_point);
            }
                // 2. downsample new cloud (save memory)
                pcl::PointCloud<PointXYZIRT>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointXYZIRT>());
                static pcl::VoxelGrid<PointXYZIRT> downSizeFilter;
                downSizeFilter.setLeafSize(0.2, 0.2, 0.1);
                downSizeFilter.setInputCloud(laserCloudIn); //
                downSizeFilter.filter(*laser_cloud_in_ds);
                std::cout<<"filter before:"<<laserCloudIn->points.size()<<",after:"<<laser_cloud_in_ds->points.size()<<std::endl;
                *laserCloudIn = *laser_cloud_in_ds;
            
        } 
        else {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        //点云校验，非RSHelios32 雷达
        if(sensor != SensorType::RSHelios)
        {
            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
            // std::cout<<"当前点云帧开始时间："<<timeScanCur<<",持续时间:"<<laserCloudIn->points.back().time<<std::endl;

            // check dense flag
            if (laserCloudIn->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }

            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }

            // check point time
            //点的时间校验
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (auto &field : currentCloudMsg.fields)
                {
                    if (field.name == "time" || field.name == "t")
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1) //点无时间戳信息
                {
                    std::cout<<"点云中的点无时间信息"<<std::endl;
                    // ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
                    
                }
            }

        }
        else //针对RSHelios 雷达点云的校验
        {
            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
            // std::cout<<"当前点云帧开始时间："<<timeScanCur<<",持续时间:"<<laserCloudIn->points.back().time<<std::endl;

            // check dense flag
            if (laserCloudIn->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }

            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        std::cout<<"点云中有线数信息"<<std::endl;
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    std::cout<<"点云中无线数信息"<<std::endl;
                    ros::shutdown();
                }
            }

            // check point time
            //点的时间校验
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (auto &field : pl_orig_RSHelios.points)
                {
                    if (abs(field.timestamp - pl_orig_RSHelios.points[0].timestamp) >= 1e-6)
                    {
                        std::cout<<"点云中的点有时间信息"<<std::endl;
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1) //点无时间戳信息
                {
                    std::cout<<"点云中的点无时间信息"<<std::endl;
                    // ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
                    
                }
            }
        }

        return true;
    }

    //激光点云去畸变
    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);
        std::lock_guard<std::mutex> lock3(odoVIOLock);

        // make sure IMU data available for the scan，只有满足第一帧IMU在激光前面，最后一帧IMU在激光帧后面才行
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }



        imuDeskewInfo();//IMU

        odomDeskewInfo();

        odomVIODeskewInfo();

        // std::cout<<"deskewInfo end"<<std::endl;

        return true;
    }

    
    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;


        //确保激光帧前面有IMU帧，IMU帧范围：[timeScanCur - 0.01,timeScanEnd + 0.01]
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        //这里直接是赋值的IMU，没有进行预计分？
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            if (imuType) {
                // get roll, pitch, and yaw estimation for this scan
                //取出激光帧前面最近IMU的RPY，赋值到要发布的点云数据结构上
                if (currentImuTime <= timeScanCur)
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
            }

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            //第0帧的IMU积分
            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;

                last_angular_x = thisImuMsg.angular_velocity.x;
                last_angular_y = thisImuMsg.angular_velocity.y;
                last_angular_z = thisImuMsg.angular_velocity.z;

                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            //IMU的旋转角度积分
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1]; //与上一帧IMU之间的时间差

            // //直接用当前帧IMU的速度代替当前帧与上一帧之间的IMU速度，误差太大了吧
            // imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            // imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            // imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;

            /****************20250822：改为用两帧之间的平均值计算********************/
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + 0.5*(angular_x+last_angular_x) * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + 0.5*(angular_y+last_angular_y) * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + 0.5*(angular_z+last_angular_z) * timeDiff;
            /*******************************************************************/

            imuTime[imuPointerCur] = currentImuTime;
            last_angular_x = angular_x;
            last_angular_y = angular_y;
            last_angular_z = angular_z;

            ++imuPointerCur;
        }

        --imuPointerCur; //因为在循环的时候，最后++，所以这里要--，才和索引一致

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    //激光里程计信息处理
    void odomDeskewInfo()
    {
        //取用IMUpreintegration节点的IMU与后端里程计融合的结果
        cloudInfo.odomAvailable = false;
        static float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20; //同步时间差

        //同样，去除当前激光帧前面的里程计数据
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - sync_diff_time)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;
        //获取距离激光帧最近的里程计帧,取出第一帧大于等于当前激光帧时间的里程计
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        //给每帧激光数据添加里程计信息
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;
        //获取激光帧后面的最近一帧里程计数据
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        //前后两帧里程计的协方差是否一致，如果一致，则认为里程计没有跳变
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;//前后帧里程计之间的位姿矩阵

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    //视觉里程计信息处理，只用了视觉里程计的初值
    void odomVIODeskewInfo()
    {
        cloudInfo.odomVIOAvailable = false;
        static float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20;
        while (!odomVIOQueue.empty())
        {
            if (odomVIOQueue.front().header.stamp.toSec() < timeScanCur - sync_diff_time)
                odomVIOQueue.pop_front();
            else
                break;
        }

        if (odomVIOQueue.empty())
            return;

        if (odomVIOQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomVIOQueue.size(); ++i)
        {
            startOdomMsg = odomVIOQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        //当前激光帧对应的视觉里程计初值
        cloudInfo.odomX = startOdomMsg.pose.pose.position.x;
        cloudInfo.odomY = startOdomMsg.pose.pose.position.y;
        cloudInfo.odomZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.odomRoll  = roll;
        cloudInfo.odomPitch = pitch;
        cloudInfo.odomYaw   = yaw;
        cloudInfo.odomResetId = (int)round(startOdomMsg.pose.covariance[0]);

        cloudInfo.odomVIOAvailable = true;

        // <!-- lviorf_yjz_lucky_boy -->
        // may be the more accurate pose is imu pre-integration--IMU预积分更准
        // get end odometry at the end of the scan
        odomVIODeskewFlag = false;
/* 
        if (odomVIOQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomVIOQueue.size(); ++i)
        {
            endOdomMsg = odomVIOQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomVIODeskewFlag = true; 
*/
    }

    //加权中值
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }

        // std::cout<<"旋转畸变处理"<<std::endl;
    }

    //赋值成了0
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        //***********20250822放开：同一帧激光点云中的点在去畸变时也考虑位置的变化**************//
        if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
            return;
        
        if(deskewFlag == 1)
        {
            // std::cout<<"时间差："<<(timeScanEnd - timeScanCur)<<std::endl;
            float ratio = relTime / (timeScanEnd - timeScanCur);

            // *posXCur = ratio * odomIncreX;
            // *posYCur = ratio * odomIncreY;
            // *posZCur = ratio * odomIncreZ;
        }
        /******************************************************************************/
    }

    //真正的激光点云去畸变函数
    PointType deskewPoint(PointType *point, double relTime)
    {
        //不进行畸变处理
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        //取得是IMU的旋转预计分，激光里程计的位置
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        //把两帧之间的位置平移当作了0
        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            //该帧的第一个点
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        //对点进行位姿变换，变换到第一个点上
        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        

        return newPoint;
    }

    //对点云进行投影，即去畸变处理
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = common_lib_->pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // if (rowIdn % downsampleRate != 0)
            //     continue;

            // if (i % point_filter_num != 0)
            //     continue;

            //对单个点云进行了坐标变换，默认了同一帧的两个点之间的位置平移为0，只进行了旋转（取的是IMU旋转）
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            fullCloud->push_back(thisPoint);
        }
    }
    
    //发布点云
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lviorf");

    common_lib_ = std::make_shared<CommonLib::common_lib>("mapping");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
