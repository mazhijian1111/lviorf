#include "utility.h"
#include "lviorf/cloud_info.h"
#include "lviorf/save_map.h"
// <!-- lviorf_yjz_lucky_boy -->
#include <sensor_msgs/NavSatFix.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;


class mapOptimization : public ParamServer
{

public:

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;
    ros::Publisher pubGPSPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::Publisher pubSLAMInfo;
    ros::Publisher pubGpsOdom;
    ros::Publisher pubCurrentPose;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subGPSOdom;
    ros::Subscriber subLoop;

    ros::ServiceServer srvSaveMap;

    std::deque<nav_msgs::Odometry> gpsQueue; //来自gps经纬高
    std::deque<nav_msgs::Odometry> gpsQueueOdom; //直接来自GPS里程计
    lviorf::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    GeographicLib::LocalCartesian gps_trans_;

    geometry_msgs::PoseStamped currentPose;

    gtsam::Pose3 vio_pose_last;//视觉里程计信息
    int index_for_vio_factor = 0;

    PointType lastGPSPoint; //上一激光帧最近的的GPS位姿
    double CurGPSTime,LastGPSTime;//当前帧与上一帧的GPS时间戳
    bool firstGPSPoint = false;

    //滤波使用
    std::deque<sensor_msgs::NavSatFix> nav_sat_fix_msg_deque;
    nav_msgs::Path GPSPath;

    //雷达关键帧时间戳
    double Curlidarkeyframestamp,Lastlidarkeyframestamp;

    int keyframe = 0;

    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("lviorf/mapping/trajectory", 1);
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lviorf/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lviorf/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lviorf/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("lviorf/mapping/path", 1);
        pubGPSPath                  = nh.advertise<nav_msgs::Path>("lviorf/mapping/gps_path", 1);

        subCloud = nh.subscribe<lviorf::cloud_info>("lviorf/deskew/cloud_info", 10, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS   = nh.subscribe<sensor_msgs::NavSatFix> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        
        //20250722订阅融合后的激光里程计
        // subGPSOdom = nh.subscribe<nav_msgs::Odometry> ("/gps_imu_odometry", 200, &mapOptimization::gpsOdomHandler, this, ros::TransportHints().tcpNoDelay());
        subGPSOdom = nh.subscribe<nav_msgs::Odometry> ("/gnss_calibrated", 200, &mapOptimization::gpsOdomHandler, this, ros::TransportHints().tcpNoDelay());
        
        
        subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("/lviorf/vins/loop/match_frame", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        srvSaveMap  = nh.advertiseService("lviorf/save_map", &mapOptimization::saveMapService, this);

        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("lviorf/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("lviorf/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lviorf/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("lviorf/mapping/map_local", 1);
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("lviorf/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lviorf/mapping/cloud_registered_raw", 1);

        pubSLAMInfo           = nh.advertise<lviorf::cloud_info>("lviorf/mapping/slam_info", 1);
        pubGpsOdom            = nh.advertise<nav_msgs::Odometry> ("lviorf/mapping/gps_odom", 1);
        pubCurrentPose        = nh.advertise<geometry_msgs::PoseStamped> ("/current_pose", 1);

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize/2);
        downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize/2);
        downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize/2);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity/2); // for surrounding key poses of scan-to-map optimization

        allocateMemory();
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const lviorf::cloud_infoConstPtr& msgIn)
    {
        // extract time stamp
        std::cout<<"后端优化点云帧回调函数"<<std::endl;
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();
        Curlidarkeyframestamp = timeLaserInfoCur;

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudSurfLast);

        // TODO
        // ......
        // remapping
        // ......
        // END

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            //1.每次进行图优化前获取初始估计
            updateInitialGuess();

            //2.查找上一帧附近50米的激光帧，作为一个submap
            extractSurroundingKeyFrames();

            //3.对当前帧点云进行将采样
            downsampleCurrentScan();

            //4.
            scan2MapOptimization();

            saveKeyFramesAndFactor();

            correctPoses();

            publishOdometry();

            publishFrames();
        }

        currentPose.header.stamp = ros::Time::now();
        currentPose.header.frame_id = "map";
        pubCurrentPose.publish(currentPose); // publish current pose
        // std::cout<<"current pose:"<<currentPose.pose.position<<","<<currentPose.pose.orientation<<std::endl;

    }

    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg)
    {
        // std::cout<<"GPS回调"<<std::endl;
        if (gpsMsg->status.status == 0)
            return;

        Eigen::Vector3d trans_local_;
        static bool first_gps = false;
        if (!first_gps) {
            first_gps = true;
            gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
            std::cout<<"GPS init: "<<gpsMsg->latitude<<" "<<gpsMsg->longitude<<" "<<gpsMsg->altitude<<std::endl;
        }

        //增加GPS滤波机制
        nav_sat_fix_msg_deque.push_back(*gpsMsg);
        if(nav_sat_fix_msg_deque.size() >= 2)
        {
            double d1 = nav_sat_fix_msg_deque[nav_sat_fix_msg_deque.size()-1].altitude - nav_sat_fix_msg_deque[nav_sat_fix_msg_deque.size()-2].altitude;
            double d2 = nav_sat_fix_msg_deque[nav_sat_fix_msg_deque.size()-1].longitude - nav_sat_fix_msg_deque[nav_sat_fix_msg_deque.size()-2].longitude;
            double d3 = nav_sat_fix_msg_deque[nav_sat_fix_msg_deque.size()-1].latitude - nav_sat_fix_msg_deque[nav_sat_fix_msg_deque.size()-2].latitude;
            if(abs(d1) >= 1.0 || abs(d2) >= 0.01 || abs(d3) >= 0.01)
            {
                std::cout<<"GPS跳变"<<std::endl;
                nav_sat_fix_msg_deque.pop_back();
                return;
            }
            nav_sat_fix_msg_deque.pop_front();
        }

        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        nav_msgs::Odometry gps_odom;
        gps_odom.header = gpsMsg->header;
        gps_odom.header.frame_id = "map";
        gps_odom.pose.pose.position.x = trans_local_[0];
        gps_odom.pose.pose.position.y = trans_local_[1];
        gps_odom.pose.pose.position.z = trans_local_[2];
        gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0); //无姿态
        pubGpsOdom.publish(gps_odom);
        gpsQueue.push_back(gps_odom);
        // std::cout<<"gpsQueue.size = "<<gpsQueue.size()<<std::endl;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = gpsMsg->header.stamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = trans_local_[0];
        pose_stamped.pose.position.y = trans_local_[1];
        pose_stamped.pose.position.z = trans_local_[2];
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

        GPSPath.poses.push_back(pose_stamped);
        GPSPath.header.stamp = gpsMsg->header.stamp;
        GPSPath.header.frame_id = odometryFrame;
        if(GPSPath.poses.size()%20 == 0)
        {
            // std::cout<<"pub GPS path, size() = "<<GPSPath.poses.size()<<std::endl;
            pubGPSPath.publish(GPSPath);
        }
    }


    //20250722增加订阅融合后的gps_imu里程计
    void gpsOdomHandler(const nav_msgs::OdometryConstPtr& gpsOdomMsg)
    {
        gpsQueueOdom.push_back(*gpsOdomMsg);
    }

    //将点经过位姿变换，变换到map系下
    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    //对点云进行位姿变换
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    //转为gtsam::Pose3格式
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    













    bool saveMapService(lviorf::save_mapRequest& req, lviorf::save_mapResponse& res)
    {
      string saveMapDirectory;

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files ..." << endl;
      if(req.destination.empty()) saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
      else saveMapDirectory = std::getenv("HOME") + req.destination;
      cout << "Save destination: " << saveMapDirectory << endl;
      // create directory and remove old files;
      int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
      unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
      // save key frame transformations
      pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
      // extract global point cloud map

      pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
      for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
          *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
          cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
      }

      if(req.resolution != 0)
      {
        cout << "\n\nSave resolution: " << req.resolution << endl;
        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
      }
      else
      {

        // save surf cloud
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
      }

      // save global point cloud map
      *globalMapCloud += *globalSurfCloud;

      int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
      res.success = ret == 0;

      downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files completed\n" << endl;

      return true;
    }

    void pubCurrentPoseThread()
    {
        ros::Rate rate(20);
        while (ros::ok()){
            rate.sleep();
            currentPose.header.stamp = ros::Time::now();
            currentPose.header.frame_id = "map";
            pubCurrentPose.publish(currentPose);
        }
    }





    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();


        }

        if (savePCD == false)
            return;

        lviorf::save_mapRequest  req;
        lviorf::save_mapResponse res;

        if(!saveMapService(req, res)){
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (common_lib_->pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }

    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            performLoopClosure();
            // visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        if (!visualLoopClosureEnableFlag)
          return;

        // control loop closure frequency
        static double last_loop_closure_time = -1;
        {
            // std::lock_guard<std::mutex> lock(mtx);
            if (timeLaserInfoCur - last_loop_closure_time < 5.0)
                return;
            else
                last_loop_closure_time = timeLaserInfoCur;
        }

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre, *loopMsg) == false)
          return;

        processICP(loopKeyCur, loopKeyPre);

        visualizeLoopClosure();
    }

    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;

        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;

        processICP(loopKeyCur, loopKeyPre);

        visualizeLoopClosure();
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID, const std_msgs::Float64MultiArray& loopMsg)
    {
        {
            std::lock_guard<std::mutex> lock(mtx);
            *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        }

        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        if (loopMsg.data.size() != 2)
            return false;

        double loopTimeCur = loopMsg.data[0];
        double loopTimePre = loopMsg.data[1];

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre || loopKeyPre == -1 || loopKeyCur == -1)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void processICP(const int& loopKeyCur, const int& loopKeyPre)
    {
        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

    //更新初始估计
    void updateInitialGuess()
    {
        // std::cout<<"update Initial Guess"<<std::endl;
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    /***********************************************************************************************************/
        static Eigen::Affine3f lastImuTransformation;
        //1、第一帧，使用IMU的预积分结果作为初始位姿
        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            //第一帧，使用IMU的预积分结果
            ROS_INFO("The first frame, use imu for initial pose guess");
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    /***********************************************************************************************************/
        

    /***********************************************************************************************************/
        // use vio estimation for pose guess
        //2、使用视觉里程计提供初始值
        static int odomResetId = 0;
        static bool lastVIOTransAvailable = false;
        static Eigen::Affine3f lastVIOTransformation;
        Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.odomX,    cloudInfo.odomY,     cloudInfo.odomZ, 
                                                    cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
        // std::cout<<"视觉里程计:x:"<<cloudInfo.odomX<<",y:"<<cloudInfo.odomY<<",z:"<<cloudInfo.odomZ<<std::endl;
        // 1. 计算位置距离（欧几里得距离）
        Eigen::Vector3f curr_pos = transBack.translation();
        Eigen::Vector3f last_pos = lastVIOTransformation.translation();
        double shift_distance = (curr_pos - last_pos).norm();
        // std::cout<<"相邻帧视觉里程计距离为："<<shift_distance<<std::endl;
        //如果视觉里程计增量小于2.0m，使用lio的初始位姿估计
        if(shift_distance < 2.0)
        {
            if (cloudInfo.odomVIOAvailable == true && cloudInfo.odomResetId == odomResetId)
            {
                ROS_INFO("use vio estimation for pose guess");
                // ROS_INFO("use vio estimation for initial pose guess");
                // Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.odomX,    cloudInfo.odomY,     cloudInfo.odomZ, 
                //                                                    cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
                if (lastVIOTransAvailable == false)
                {
                    lastVIOTransformation = transBack;
                    lastVIOTransAvailable = true;
                } else {
                    Eigen::Affine3f transIncre = lastVIOTransformation.inverse() * transBack; //视觉变换增量
                    Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                    Eigen::Affine3f transFinal = transTobe * transIncre; //变换到全局坐标系下
                    pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                    lastVIOTransformation = transBack;

                    lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;

                    return;
                }
            }  else {
                // ROS_WARN("VINS failure detected.");
                lastVIOTransAvailable = false;
                odomResetId = cloudInfo.odomResetId;
            }
        }else{
            lastVIOTransformation = transBack;
        }
    /***********************************************************************************************************/

    /***********************************************************************************************************/
        //如果视觉里程计提供初始位姿失败，则启用GPS提供初始位姿
        /************************20250723增加，start************************************ */
        // //use gio estimation for pose guess
        // bool is_curr_gio_avilable = true;
        // static bool lastGIOTransAvailable = false;
        // static Eigen::Affine3f lastGIOTransformation;
        // if(gpsQueue.empty())
        //    is_curr_gio_avilable = false;
        
        // while (!gpsQueue.empty())
        // {
        //     if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
        //     {
        //         // message too old
        //         gpsQueue.pop_front();
        //     }
        //     else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
        //     {
        //         // message too new
        //         is_curr_gio_avilable = false;
        //         break;
        //     }else{
        //         break;
        //     }
        // }

        // //从gpsQueue中查找与当前激光帧时间最接近的GPS点
        // double CurrLaserTime = timeLaserInfoCur;
        // double minTimeDiff = 1000000;
        // int minIndex = -1;
        // for (int i = 0; i < (int)gpsQueue.size(); ++i)
        // {
        //     double timeDiff = abs(gpsQueue[i].header.stamp.toSec() - CurrLaserTime);
        //     if (timeDiff < minTimeDiff)
        //     {
        //         minTimeDiff = timeDiff;
        //         minIndex = i;
        //     }
        // }
        
        
        // if (minIndex == -1 || minTimeDiff > 0.01) //10ms
        //   is_curr_gio_avilable = false;
        
        // PointType curGPSPoint; //当前激光帧最近的的GPS位姿
        // curGPSPoint.x = gpsQueue[minIndex].pose.pose.position.x;
        // curGPSPoint.y = gpsQueue[minIndex].pose.pose.position.y;
        // curGPSPoint.z = gpsQueue[minIndex].pose.pose.position.z;

        // // GPS too noisy, skip
        // float noise_x = gpsQueue[minIndex].pose.covariance[0];
        // float noise_y = gpsQueue[minIndex].pose.covariance[7];
        // float noise_z = gpsQueue[minIndex].pose.covariance[14];
        // if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold || noise_z > gpsCovThreshold)
        //     is_curr_gio_avilable = false;

        // if(is_curr_gio_avilable)
        // {
        //     std::cout<<"use gio estimation for pose guess"<<std::endl;
        //     //这个旋转准确吗？
        //     Eigen::Quaterniond q(gpsQueue[minIndex].pose.pose.orientation.w, 
        //                          gpsQueue[minIndex].pose.pose.orientation.x,
        //                          gpsQueue[minIndex].pose.pose.orientation.y,
        //                          gpsQueue[minIndex].pose.pose.orientation.z
        //                          );
        //     Eigen::Vector3d eulerAngle = q.toRotationMatrix().eulerAngles(2, 1, 0);//返回Vector3d(yaw, pitch, roll)
        //     Eigen::Affine3f transBack1 = pcl::getTransformation(curGPSPoint.x, curGPSPoint.y, 0.0, 
        //                                                         cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        //     if (lastGIOTransAvailable == false)
        //     {
        //         lastGIOTransformation = transBack1;
        //         lastGIOTransAvailable = true;
        //     } else {
        //         Eigen::Affine3f transIncre1 = lastGIOTransformation.inverse() * transBack1;
        //         Eigen::Affine3f transTobe1 = trans2Affine3f(transformTobeMapped);
        //         Eigen::Affine3f transFinal1 = transTobe1 * transIncre1;
        //         pcl::getTranslationAndEulerAngles(transFinal1, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
        //                                                         transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        //         lastGIOTransformation = transBack1;

        //         lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        //         return;
        //     }
        // }
        /************************20250723增加，end************************************ */
    /***********************************************************************************************************/


    /***********************************************************************************************************/
/*      
        // has bad benifit for LIO
        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                
                // debug print
                std::cout << "\033[1m\033[33m" << "Imu pre-integration predict pose >>> " << transformTobeMapped[3] << " " << transformTobeMapped[4] 
                            << " " << transformTobeMapped[5] << " " << transformTobeMapped[0] << " " << transformTobeMapped[1] 
                              << " " << transformTobeMapped[2] << "\033[0m" << std::endl;
                return;
            }
        }
 */
    /***********************************************************************************************************/

    /***********************************************************************************************************/
        // use imu incremental estimation for pose guess (only rotation)
        //imuType为0时表示6轴IMU，1时表示9轴IMU
        // std::cout<<"IMU:"<<cloudInfo.imuAvailable<<",imuType:"<<imuType<<std::endl;
        if (cloudInfo.imuAvailable == true && imuType)
        {
            ROS_INFO("use imu estimation for initial pose guess");

            //当前帧IMU估计值
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            //上一帧与当前帧激光之间的IMU增量
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            //上一帧图优化估计的结果
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;

            //作为初值
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    /***********************************************************************************************************/
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }
        std::cout<<"cloudKeyPoses3D.size = "<<cloudKeyPoses3D->size()<<",surroundingKeyPosesDS.size="<<surroundingKeyPosesDS->size()<<std::endl;
        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudSurfFromMap->clear(); //清空邻近地图帧
        
        //遍历需要提取的地图帧对应的位姿
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (common_lib_->pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp;

                //索引为thisKeyInd的激光帧（激光雷达坐标系下），变换到map坐标系下
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding surf key frames (or map)
        downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    //求解变换矩阵
    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    /**
     * 特征优化函数
     * 通过当前帧的特征点与地图中的特征进行匹配，计算特征残差和雅可比矩阵
     * 用于后续的位姿优化
     */
    void surfOptimization()
    {
        // 更新上一帧激光帧到地图坐标系的变换矩阵
        // 这个变换矩阵用于将当前帧点云转换到地图坐标系中进行匹配
        updatePointAssociateToMap();

        // 使用OpenMP进行并行计算，提高处理速度
        // numberOfCores指定使用的CPU核心数量
        #pragma omp parallel for num_threads(numberOfCores)
        //遍历下采样后的当前特征点云中的所有点，laserCloudSurfLastDSNum为当前帧下采样后的点数
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            // pointOri: 原始点（在激光雷达坐标系）
            // pointSel: 变换到地图坐标系后的点
            // coeff: 存储计算得到的平面系数和残差
            PointType pointOri, pointSel, coeff;

            // 用于存储最近邻搜索结果的索引和距离
            std::vector<int> pointSearchInd; // 最近邻点的索引
            std::vector<float> pointSearchSqDis; // 最近邻点的平方距离

            pointOri = laserCloudSurfLastDS->points[i];//从当前帧中取出第i点

            // 将当前点从激光雷达坐标系变换到地图坐标系
            // pointOri: 输入，原始坐标系下的点
            // pointSel: 输出，变换到地图坐标系后的点
            pointAssociateToMap(&pointOri, &pointSel); //将当前帧中的点变换到map坐标系下

            //从邻近帧中搜索与该点最近的5个点
            // 在地图的面特征KD树中搜索当前点的5个最近邻点
            // kdtreeSurfFromMap: 邻近帧地图特征的KD树
            // pointSel: 查询点（在地图坐标系）
            // 5: 要搜索的最近邻点数量
            // pointSearchInd: 输出，最近邻点的索引
            // pointSearchSqDis: 输出，到最近邻点的平方距离
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            // 定义矩阵用于平面拟合：
            // matA0: 5x3矩阵，存储5个最近邻点的坐标
            // matB0: 5x1矩阵，全部填充为-1，用于平面方程求解
            // matX0: 3x1向量，存储平面方程系数
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            // 初始化矩阵
            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            // 检查第5个最近邻点的距离是否小于1.0米
            // 如果距离太大，说明点太稀疏，不适合进行平面拟合
            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {// 构建矩阵A：将5个最近邻点的坐标填入matA0
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                // 使用QR分解求解线性最小二乘问题：matA0 * matX0 = matB0
                // 这里求解的是平面方程 ax + by + cz + d = 0 中的a,b,c
                // matB0全为-1相当于求解 ax + by + cz = 1 的最小二乘解
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                // 从解向量中提取平面参数
                float pa = matX0(0, 0); // 平面法向量的x分量
                float pb = matX0(1, 0); // 平面法向量的y分量
                float pc = matX0(2, 0); // 平面法向量的z分量
                float pd = 1;       // 平面方程的常数项，初始设为1

                // 计算法向量的模长，用于归一化
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps; // 归一化平面参数，得到单位法向量

                // 验证平面拟合的质量
                bool planeValid = true;
                // 计算每个最近邻点到拟合平面的距离,
                // 平面方程：pa*x + pb*y + pc*z + pd = 0
                for (int j = 0; j < 5; j++) {
                    // 如果任何一个点的距离大于0.2米，认为平面拟合不准确
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                 // 如果平面拟合质量合格，计算当前点到该平面的距离作为残差
                if (planeValid) {
                    // 计算当前点(pointSel)到拟合平面的距离
                    // 这就是面特征的残差：点到平面的距离
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    // 计算权重因子s，用于后续的优化
                    // 这个权重基于点到平面的距离和点的原始距离
                    // 距离越近，权重越大；点离雷达越远，权重调整越小
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    // 存储平面法向量和残差到coeff结构体中
                    // 法向量乘以权重s
                    coeff.x = s * pa; // 加权的平面法向量x分量
                    coeff.y = s * pb; // 加权的平面法向量y分量
                    coeff.z = s * pc; // 加权的平面法向量z分量
                    coeff.intensity = s * pd2;

                    // 如果权重s大于0.1，说明这个匹配质量较好，将其保存用于后续优化
                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri; // 保存原始点（激光坐标系）
                        coeffSelSurfVec[i] = coeff; // 保存系数和残差
                        laserCloudOriSurfFlag[i] = true; // 标记这个点有效
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);//保存原始点（激光坐标系）
                coeffSel->push_back(coeffSelSurfVec[i]); //拟合平面的系数和距离
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    /**
     * LM优化方法（Levenberg-Marquardt Optimization）
     * 使用当前帧的特征点与局部地图进行匹配，通过迭代优化求解最优位姿变换
     * @param iterCount 当前迭代次数
     * @return 如果收敛返回true，否则返回false
     */
    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // 激光雷达坐标系与相机坐标系的转换关系
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        // 计算当前位姿变换的三角函数值，用于后续雅可比矩阵计算
        // transformTobeMapped数组存储当前估计的位姿：[roll, pitch, yaw, x, y, z]
        float srx = sin(transformTobeMapped[2]); // sin(roll)
        float crx = cos(transformTobeMapped[2]); // cos(roll)
        float sry = sin(transformTobeMapped[1]); // sin(pitch) 
        float cry = cos(transformTobeMapped[1]); // cos(pitch)
        float srz = sin(transformTobeMapped[0]); // sin(yaw)
        float crz = cos(transformTobeMapped[0]); // cos(yaw)

        //当前激光帧中能与局部地图拟合出平面的点云数量
        int laserCloudSelNum = laserCloudOri->size();
        // 如果有效特征点太少（少于50个），不足以进行可靠的优化，返回失败
        if (laserCloudSelNum < 50) {
            return false;
        }


        // 初始化矩阵用于构建线性最小二乘问题：J^T * J * delta_x = -J^T * f(x)
        // matA: 雅可比矩阵J，大小 laserCloudSelNum x 6，每个点对6个自由度（roll,pitch,yaw,x,y,z）的导数
        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        // matAt: 雅可比矩阵的转置，大小 6 x laserCloudSelNum
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        // matAtA: 海森矩阵的近似 J^T * J，大小 6 x 6
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        // matB: 残差向量f(x)，大小 laserCloudSelNum x 1
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        // matAtB: J^T * f(x)，大小 6 x 1
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        // matX: 待求解的增量delta_x，大小 6 x 1，包含6个自由度的位姿增量
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        // 定义临时变量：pointOri存储原始点坐标，coeff存储平面系数和残差
        PointType pointOri, coeff;

        // 遍历所有有效特征点，构建最小二乘问题
        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            // 从激光坐标系转换到相机坐标系（根据开头的坐标转换关系）
            // 获取第i个特征点的原始坐标（激光坐标系）
            pointOri.x = laserCloudOri->points[i].x;
            pointOri.y = laserCloudOri->points[i].y;
            pointOri.z = laserCloudOri->points[i].z;
            // lidar -> camera
            // 获取第i个特征点对应的平面系数和残差
            // coeff.x, coeff.y, coeff.z: 平面单位法向量的三个分量
            // coeff.intensity: 点到平面的距离（残差）
            coeff.x = coeffSel->points[i].x;
            coeff.y = coeffSel->points[i].y;
            coeff.z = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
/*             float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
             */

            // 计算关于roll的导数分量
            float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                      + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;
            
            // 计算关于pitch的导数分量
            float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                      + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                      + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;
            // 计算关于yaw的导数分量
            float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                      + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                      + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
              
            // camera -> lidar
            // 将导数从相机坐标系转换回激光雷达坐标系，并填充到雅可比矩阵中
            // 注意：由于坐标系的差异，导数的顺序需要调整
            matA.at<float>(i, 0) = arz; // 关于yaw的导数
            matA.at<float>(i, 1) = ary; // 关于pitch的导数 
            matA.at<float>(i, 2) = arx; // 关于roll的导数
            // 关于平移的导数直接使用平面法向量
            matA.at<float>(i, 3) = coeff.x; // 关于x平移的导数
            matA.at<float>(i, 4) = coeff.y; // 关于y平移的导数
            matA.at<float>(i, 5) = coeff.z; // 关于z平移的导数

            // 残差项：负的点到平面距离（因为我们要最小化这个距离）
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        // 构建正规方程：J^T * J * delta_x = -J^T * f(x)
        cv::transpose(matA, matAt); // 计算雅可比矩阵的转置
        matAtA = matAt * matA;   // 计算海森矩阵近似 J^T * J
        matAtB = matAt * matB;   // 计算 J^T * f(x)

        // 使用QR分解求解线性方程组：matAtA * matX = matAtB
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        // 只在第一次迭代时进行退化检测
        if (iterCount == 0) {
            // 初始化特征值分解相关矩阵
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0)); // 特征值矩阵
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0)); // 特征向量矩阵
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0)); // 处理后的特征向量矩阵

            // 对海森矩阵进行特征值分解：matAtA = matV * matE * matV^T
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);// 备份特征向量矩阵

            // 退化检测：检查是否有特征值过小（接近奇异）
            isDegenerate = false;
            // 设置各自由度的特征值阈值
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            // 从最小的特征值开始检查（特征值按降序排列）
            for (int i = 5; i >= 0; i--) {
                // 如果特征值小于阈值，说明该自由度方向上的约束较弱
                if (matE.at<float>(0, i) < eignThre[i]) {
                    // 将该特征值对应的特征向量置零，避免在退化方向上更新
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true; // 标记存在退化情况
                } else {
                    // 一旦遇到大于阈值的特征值，就停止检查（特征值按降序排列）
                    break;
                }
            }
            // 计算投影矩阵，用于在退化情况下约束解的空间
            matP = matV.inv() * matV2;
        }

        // 如果存在退化情况，使用投影矩阵约束解
        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2); // 备份原始解
            matX = matP * matX2; // 投影到非退化子空间
        }

        // 更新位姿估计：x_{k+1} = x_k + delta_x
        transformTobeMapped[0] += matX.at<float>(0, 0); // 更新yaw
        transformTobeMapped[1] += matX.at<float>(1, 0); // 更新pitch
        transformTobeMapped[2] += matX.at<float>(2, 0); // 更新roll
        transformTobeMapped[3] += matX.at<float>(3, 0); // 更新x平移
        transformTobeMapped[4] += matX.at<float>(4, 0); // 更新y平移  
        transformTobeMapped[5] += matX.at<float>(5, 0); // 更新z平移

        // 计算本次迭代的位姿变化量，用于收敛判断
        // 计算旋转变化量（将弧度转换为角度）
        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) + // yaw变化
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) + // pitch变化
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        // 计算平移变化量（将米转换为厘米）
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));
        
        
        // 收敛判断：如果旋转和平移的变化量都很小，认为已经收敛
        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }


    //计算位姿
    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudSurfLastDSNum > 30)
        {
            // std::cout<<"通过scan 2 submap 计算位姿"<<std::endl;

            //将邻近帧放到KD树中
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                //求当前帧的点与局部地图拟合的平面系数
                surfOptimization();

                combineOptimizationCoeffs();

                //执行优化
                if (LMOptimization(iterCount) == true)
                    break;              
            }
            std::cout<<"前端计算得到的位姿,x:"<<transformTobeMapped[3]<<",y:"<<transformTobeMapped[4]<<",z:"<<transformTobeMapped[5]
            <<",roll:"<<transformTobeMapped[2]<<",pitch:"<<transformTobeMapped[1]<<",yaw:"<<transformTobeMapped[0]<<std::endl;

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d planar features available.", laserCloudSurfLastDSNum);
        }
    }

    /**
     * 变换更新函数
     * 主要功能：使用IMU数据对激光SLAM的位姿估计进行修正和约束，提高姿态估计的准确性
     * 特别针对俯仰角(pitch)和横滚角(roll)进行IMU融合，并对变换参数进行合理性约束
     */
    void transformUpdate()
    {
        // 检查IMU数据是否可用且IMU类型有效
        if (cloudInfo.imuAvailable == true && imuType)
        {
            // 只有当初始俯仰角的绝对值小于1.4弧度（约80度）时才使用IMU数据进行姿态修正
            // 这个条件是为了避免在极端姿态下使用可能不可靠的IMU数据
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                // 设置IMU数据的权重，这个权重决定了IMU数据在融合中的影响力
                // imuRPYWeight通常在0-1之间，值越大表示越信任IMU数据
                double imuWeight = imuRPYWeight;
                // 定义四元数变量用于姿态插值计算
                tf::Quaternion imuQuaternion; // IMU测量得到的姿态四元数
                tf::Quaternion transformQuaternion; // 当前SLAM估计的姿态四元数
                double rollMid, pitchMid, yawMid; // 存储插值后的欧拉角

                // slerp roll
                // 对横滚角(roll)进行球面线性插值(Slerp)融合
                // 步骤1：将SLAM估计的roll角转换为四元数（pitch和yaw设为0）
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                // 步骤2：将IMU测量的roll角转换为四元数
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                // 步骤3：在两个四元数之间进行球面线性插值，然后将结果转换回欧拉角
                // slerp函数参数：目标四元数，插值权重（0=完全使用transform，1=完全使用IMU）
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                // 步骤4：用融合后的roll角更新SLAM的位姿估计
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                // 对俯仰角(pitch)进行球面线性插值(Slerp)融合
                // 步骤1：将SLAM估计的pitch角转换为四元数（roll和yaw设为0）
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                // 步骤2：将IMU测量的pitch角转换为四元数
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                // 步骤3：在两个四元数之间进行球面线性插值
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                // 步骤4：用融合后的pitch角更新SLAM的位姿估计
                transformTobeMapped[1] = pitchMid;

                // 注意：这里没有对偏航角(yaw)进行IMU融合，因为IMU的yaw角通常存在漂移问题
                // 而激光SLAM在yaw角估计上通常更加可靠
            }
        }

        // 对变换参数进行约束，确保其在合理的物理范围内
        // 约束横滚角(roll)，防止出现不合理的旋转值
        // transformTobeMapped[0] = roll, rotation_tollerance是旋转角度的容差阈值
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        // 约束俯仰角(pitch)，防止出现不合理的旋转值
        // transformTobeMapped[1] = pitch
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        // 约束Z轴高度(z)，防止出现不合理的高度变化
        // transformTobeMapped[5] = z, z_tollerance是高度变化的容差阈值
        // 这对于地面机器人特别重要，可以避免估计出不合理的高度跳跃
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
        
        // 将更新后的6自由度变换参数转换为Affine3f变换矩阵，并保存到增量里程计中
        // incrementalOdometryAffineBack用于存储当前帧相对于上一帧的位姿变换
        // 这个变换矩阵将在后续的帧间匹配和地图更新中使用
        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    //通过上一帧与当前帧的位姿变化值，判断是否需要添加新的关键帧
    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());//上一帧图优化的结果

        //当前前端计算得到的位姿
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        // std::cout<<"saveFrame()?，两帧之间距离为:"<<sqrt(x*x + y*y + z*z)<<","<<abs(roll)<<","<<abs(pitch)<<","<<abs(yaw)<<std::endl;

        //通过时间判断是否添加关键帧,相差0.5s为关键帧
        if(abs(Curlidarkeyframestamp - Lastlidarkeyframestamp) > 0.5)
        {
            std::cout<<"检测到关键帧，两关键帧之间的时间差为:"<<Curlidarkeyframestamp - Lastlidarkeyframestamp<<std::endl;
            Lastlidarkeyframestamp = Curlidarkeyframestamp;
            return true;
        }

        if ((abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold))
            return false;

        std::cout<<"检测到关键帧，两帧之间距离为:"<<sqrt(x*x + y*y + z*z)<<std::endl;
        Lastlidarkeyframestamp = Curlidarkeyframestamp;
        return true;
    }

    //在上一帧与当前帧之间添加激光里程计因子
    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            // if (initialEstimate.exists(0)) {
            // // 键已存在，更新值而不是插入
            // initialEstimate.update(0, trans2gtsamPose(transformTobeMapped));
            // std::cout << "Updated existing key: " << cloudKeyPoses3D->size() << std::endl;
            // } else {
            //     // 键不存在，插入新值
            //     initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
            //     std::cout << "Inserted new key: " << cloudKeyPoses3D->size() << std::endl;
            // }
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); //上一帧
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped); //当前帧估计
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            // if (initialEstimate.exists(cloudKeyPoses3D->size())) {
            // // 键已存在，更新值而不是插入
            // initialEstimate.update(cloudKeyPoses3D->size(), poseTo);
            // std::cout << "Updated existing key: " << cloudKeyPoses3D->size() << std::endl;
            // } else {
            //     // 键不存在，插入新值
            //     initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
            //     std::cout << "Inserted new key: " << cloudKeyPoses3D->size() << std::endl;
            // }
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
            std::cout<<"add lio factor"<<std::endl;
        }
    }


    //视觉里程计因子，20250909增加
    void addVIOFactor()
    {
        static int odomResetId1 = 0;
        static bool lastVIOAvailable = false;
        gtsam::Pose3 vio_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(double(cloudInfo.odomRoll), double(cloudInfo.odomPitch), double(cloudInfo.odomYaw)),
                                  gtsam::Point3(double(cloudInfo.odomX),    double(cloudInfo.odomY),     double(cloudInfo.odomZ)));                
        
        double shift_distance = sqrt(pow(vio_pose.x()-vio_pose_last.x(),2)+pow(vio_pose.y()-vio_pose_last.y(),2)+pow(vio_pose.z()-vio_pose_last.z(),2));

        //如果视觉里程计增量小于2.0m，添加视觉里程计因子
        // std::cout<<"视觉里程计因子距离为："<<shift_distance<<std::endl;
        if(shift_distance < 2.0)
        {
            if (cloudInfo.odomVIOAvailable == true && cloudInfo.odomResetId == odomResetId1)
            {

                if (lastVIOAvailable == false)
                {
                    index_for_vio_factor = cloudKeyPoses3D->size();
                    vio_pose_last = vio_pose;
                    lastVIOAvailable = true;
                } else {

                    if(cloudKeyPoses3D->points.empty())
                    {
                        // noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
                        // gtSAMgraph.add(PriorFactor<Pose3>(0, vio_pose, priorNoise));
                        // initialEstimate.insert(0, vio_pose);
                        // index_for_vio_factor = cloudKeyPoses3D->size();
                    }else{
                        // ROS_INFO("add vio factor");
                        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1).finished());
                        gtsam::Pose3 poseFrom = vio_pose_last; //上一帧视觉估计位姿
                        gtsam::Pose3 poseTo   = vio_pose; //当前帧视觉估计
                        gtSAMgraph.add(BetweenFactor<Pose3>(index_for_vio_factor, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
                        index_for_vio_factor = cloudKeyPoses3D->size();
                    }
                    vio_pose_last = vio_pose;
                    return;
                }
            }else{
                index_for_vio_factor = cloudKeyPoses3D->size();
                odomResetId1 = cloudInfo.odomResetId;
                vio_pose_last = vio_pose;
                lastVIOAvailable = false;
            }
        }else{
            index_for_vio_factor = cloudKeyPoses3D->size();
            vio_pose_last = vio_pose;
        }
    }




    //新的添加GPS因子的机制,20250722增加，20250910修改
    void addGPSFactorNew()
    {
        gpsQueueOdom = gpsQueue;
        // std::cout<<"start add GPS Factor New"<<std::endl;
        if (gpsQueueOdom.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            // if (common_lib_->pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 1.0)
                // return;
        }
        // pose covariance small, no need to correct
        // if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        //     return;

        std::cout<<"gpsQueueOdom.size = "<<gpsQueueOdom.size()<<std::endl;
        
        while (!gpsQueueOdom.empty())
        {
            if (gpsQueueOdom.front().header.stamp.toSec() < timeLaserInfoCur - 0.1)
            {
                // message too old
                gpsQueueOdom.pop_front();
            }
            else if (gpsQueueOdom.front().header.stamp.toSec() > timeLaserInfoCur + 0.1)
            {
                // message too new
                return;
            }else{
                break;
            }
        }


        //优化前的位置
        PointType curLaserPoint;
        curLaserPoint.x = transformTobeMapped[3];
        curLaserPoint.y = transformTobeMapped[4];
        curLaserPoint.z = transformTobeMapped[5];
        
        //从gpsQueue中查找与当前激光帧时间最接近的GPS点
        double CurrLaserTime = timeLaserInfoCur;
        double minTimeDiff = 1000000;
        int minIndex = -1;
        for (int i = 0; i < (int)gpsQueueOdom.size(); ++i)
        {
            std::cout<<setprecision(15)<<"gpsQueueOdom:"<<gpsQueueOdom[i].header.stamp.toSec()<<",lidattime:"<<CurrLaserTime<<std::endl;
            double timeDiff = abs(gpsQueueOdom[i].header.stamp.toSec() - CurrLaserTime);
            if (timeDiff < minTimeDiff)
            {
                minTimeDiff = timeDiff;
                minIndex = i;
            }
        }
        
        std::cout<<"minIndex:"<<minIndex<<",minTimeDiff:"<<minTimeDiff<<std::endl;
        if (minIndex == -1 || minTimeDiff > 0.02)
          return;
        
        PointType curGPSPoint; //当前激光帧最近的的GPS位姿
        curGPSPoint.x = gpsQueueOdom[minIndex].pose.pose.position.x;
        curGPSPoint.y = gpsQueueOdom[minIndex].pose.pose.position.y;
        curGPSPoint.z = gpsQueueOdom[minIndex].pose.pose.position.z;
        CurGPSTime = gpsQueueOdom[minIndex].header.stamp.toSec();

        // GPS too noisy, skip
        float noise_x = gpsQueueOdom[minIndex].pose.covariance[0];
        float noise_y = gpsQueueOdom[minIndex].pose.covariance[7];
        float noise_z = gpsQueueOdom[minIndex].pose.covariance[14];
        std::cout<<"noise_x:"<<noise_x<<" noise_y:"<<noise_y<<" noise_z:"<<noise_z<<std::endl;
        if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
            return;

        float gps_x = gpsQueueOdom[minIndex].pose.pose.position.x;
        float gps_y = gpsQueueOdom[minIndex].pose.pose.position.y;
        float gps_z = 0.0;
        if (!useGpsElevation)
        {
            gps_z = transformTobeMapped[5];
            noise_z = 0.01;
        }

        //如果协方差太大


        // GPS not properly initialized (0,0,0)
        if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
            return;

        if(!firstGPSPoint)
        {
            lastGPSPoint = curGPSPoint;
            LastGPSTime = CurGPSTime;
            firstGPSPoint = true;
            return;
        }else{
            // Add GPS every a few meters
            double shift_distance = common_lib_->pointDistance(lastGPSPoint, curGPSPoint);
            // std::cout<<"GPS里程计因子距离为:"<<shift_distance<<std::endl;

            // if (abs(CurGPSTime-LastGPSTime)>3.0 && abs(curGPSPoint.x - lastGPSPoint.x) < 5.0 &&
            // abs(curGPSPoint.y - lastGPSPoint.y) < 5.0 &&
            // abs(curGPSPoint.z - lastGPSPoint.z)< 5.0)
            {
                // Add GPS factor
                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);
                lastGPSPoint = curGPSPoint;
                LastGPSTime = CurGPSTime;

                std::cout<<"add new GPS factor!"<<std::endl;
            }    

            // // Add GPS factor
            // gtsam::Vector Vector3(3);
            // Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            // noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
            // gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            // gtSAMgraph.add(gps_factor);
            // lastGPSPoint = curGPSPoint;
            // LastGPSTime = CurGPSTime;

            // std::cout<<"add new GPS factor!"<<std::endl;
        }

    }



    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            // if (common_lib_->pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            //     return;
        }

        // pose covariance small, no need to correct
        // if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        //     return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.01)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.05)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                // if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                //     continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;

                //两帧GPS位置相差3.0米以上，才添加GPS因子
                // if (common_lib_->pointDistance(curGPSPoint, lastGPSPoint) < 3.0)
                //     continue;
                // else
                    lastGPSPoint = curGPSPoint;
                

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                std::cout<<"add GPS factor"<<std::endl;
                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }
        std::cout<<"add loop factor"<<std::endl;

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
        {
            std::cout<<"当前激光帧为非关键帧"<<std::endl;
            return;
        }
        // std::cout<<"当前激光帧为关键帧"<<std::endl;
        cout << "*************************Begin backend optimization***************************" << endl;
        std::cout<<"keyframe:"<<keyframe<<std::endl;

        // lio odom factor
        addOdomFactor();

        //vio factor
        addVIOFactor();

        // gps factor
        // addGPSFactor();

        // addGPSFactorNew();

        // loop factor
        addLoopFactor();

        
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        
        // gtSAMgraph.resize(0);
        initialEstimate.clear();
        // 定期清理 iSAM2 内部状态
        if(keyframe > 0 && keyframe % 100 == 0)
        {
            keyframe = 0;
            gtSAMgraph.resize(0);
            // isam->clear(); // 或者使用更精细的清理策略
            // 
            // 
        }


        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;


        //最新的估计
        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        //保存优化更新后的位姿
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud，存入到关键帧队列中
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        //更新轨迹
        updatePath(thisPose6D);


        /***************************20250801：转换全局定位结果******************************** */
        //利用初始帧的gps坐标在全局坐标系中的位姿，将当前帧的gps坐标转换为全局坐标系下
        //手动设置一个全局坐标系原点的经纬度，系统启动的第一帧经纬度作为系统建图原点
        double globalOriginLat = 39.984060;
        double globalOriginLon = 116.307520;
        //设置当前帧的gps坐标
        double currentLat = 39.984060;
        double currentLon = 116.307520;
        //将当前帧的gps坐标转换为全局坐标系下
        double currentX = 0.0;
        double currentY = 0.0;
        double currentZ = 0.0;
        //将当前帧的gps坐标转换为全局坐标系下
        // gpsToGlobal(currentLat, currentLon, globalOriginLat, globalOriginLon, currentX, currentY, currentZ);
        //将当前帧的gps坐标转换为全局坐标系下
        // transformTobeMapped[3] = currentX;
        // transformTobeMapped[4] = currentY;
        // transformTobeMapped[5] = currentZ;
        currentPose.header.stamp = ros::Time().fromSec(timeLaserInfoCur);
        currentPose.header.frame_id = "map";

        currentPose.pose.position.x = latestEstimate.translation().x();
        currentPose.pose.position.y = latestEstimate.translation().y();
        currentPose.pose.position.z = latestEstimate.translation().z();
        tf::Quaternion q = tf::createQuaternionFromRPY(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());
        currentPose.pose.orientation.x = q.x();
        currentPose.pose.orientation.y = q.y();
        currentPose.pose.orientation.z = q.z();
        currentPose.pose.orientation.w = q.w();




        //发布当前帧的激光定位结果
        pubCurrentPose.publish(currentPose);
        /***************************20250801：转换全局定位结果******************************** */

        keyframe++;
        cout << "*************************End backend optimization***************************" << endl;
    }

    //如果检测到闭环，则更新关键帧位姿，并更新轨迹
    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        //如果有闭环，则对位姿和轨迹做一次更新
        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
        ROS_INFO("pose after optimize (x,y,z):%f,%f,%f",pose_in.x,pose_in.y,pose_in.z);
    }

    //发布激光里程计
    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);
        // std::cout<<"publish TF from "<<odometryFrame<<" --> lidar_link"<<std::endl;
        // std::cout<<"lio:"<<transformTobeMapped[3]<<" "<<transformTobeMapped[4]<<" "<<transformTobeMapped[5]<<std::endl;
        // ROS_INFO("lio_xyz : %f, %f, %f", transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);



        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true && imuType)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo.getNumSubscribers() != 0)
        {
            if ((int)lastSLAMInfoPubSize != (int)cloudKeyPoses6D->size())
            {
                lviorf::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lviorf");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    // std::thread CurrentPoseThread(&mapOptimization::pubCurrentPoseThread, &MO);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();
    // CurrentPoseThread.join();

    return 0;
}
