#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0


// mtx lock for two threads
std::mutex mtx_lidar;

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

// deque<pcl::PointCloud<PointType>> cloudQueue_trans;
// deque<Eigen::Affine3f> transform_offset_deque;
// deque<Eigen::Affine3f> transform_now_deque;
// deque<Eigen::Affine3f> lio_transform_deque;

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

// feature publisher for VINS estimator
ros::Publisher pub_feature;
ros::Publisher pub_match;
ros::Publisher pub_restart;
ros::Publisher pub_test;

// feature tracker variables
FeatureTracker trackerData[NUM_OF_CAM]; //特征点跟踪数组
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;


void imgCompressed_callback(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    // std::cout<<"订阅到图像帧"<<std::endl;
    double cur_img_time = img_msg->header.stamp.toSec();

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }


       
    cv::Mat cv_ptr;
    cv_bridge::CvImageConstPtr ptr;
    // * ROS消息格式转cv::Mat
    ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++) //读入图像
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time);
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
    }

    // std::cout<<"光流提取完成"<<std::endl;

    //更新特征点ID
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        id_of_point.name = "feature_id";
        u_of_point.name = "u_of_point";
        v_of_point.name = "v_of_point";
        velocity_x_of_point.name = "velocity_x_of_point";
        velocity_y_of_point.name = "velocity_y_of_point";

        feature_points->header.stamp = img_msg->header.stamp;
        feature_points->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                // std::cout<<"ID为"<<j<<"的特征点被跟踪"<<trackerData[i].track_cnt[j]<<"次"<<std::endl;
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    // std::cout<<"ID为"<<j<<"的特征点被跟踪"<<trackerData[i].track_cnt[j]<<"次,空间坐标为："<<un_pts[j].x<<","<<un_pts[j].y
                    // <<"像素坐标为："<<cur_pts[j].x<<","<<cur_pts[j].y<<std::endl;
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        // get feature depth from lidar point cloud
        //读取激光雷达点云
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();

        //获取深度信息
        sensor_msgs::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points->points);
        feature_points->channels.push_back(depth_of_points);
        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature.publish(feature_points);

        std::cout<<"视觉跟踪到的特征点数量："<<feature_points->points.size()<<std::endl;

        // publish features in image
        if (pub_match.getNumSubscribers() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            pub_match.publish(ptr->toImageMsg());
        }
    }
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double cur_img_time = img_msg->header.stamp.toSec();

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++) //读入图像
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time);
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
    }
    std::cout<<"光流提取完成"<<std::endl;

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        id_of_point.name = "feature_id";
        u_of_point.name = "u_of_point";
        v_of_point.name = "v_of_point";
        velocity_x_of_point.name = "velocity_x_of_point";
        velocity_y_of_point.name = "velocity_y_of_point";

        feature_points->header.stamp = img_msg->header.stamp;
        feature_points->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        // get feature depth from lidar point cloud
        //读取激光雷达点云
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();

        //获取深度信息
        sensor_msgs::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points->points);
        feature_points->channels.push_back(depth_of_points);
        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature.publish(feature_points);
        
        std::cout<<"视觉跟踪到的特征点数量："<<feature_points->points.size()<<std::endl;

        // publish features in image
        if (pub_match.getNumSubscribers() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 6, cv::Scalar(255 * (1 - len), 255 * len, 0), -1);
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 6, cv::Scalar(0, 255, 0), -1);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 6, cv::Scalar(0, 0, 255), -1);
                        }
                    }
                }
            }

            pub_match.publish(ptr->toImageMsg());
        }
    }
}

// void Desked_lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
// {
//     static int lidar_count = -1;
//     if (++lidar_count % (LIDAR_SKIP+1) != 0)
//         return;

//     // 0. listen to transform
//     static tf::TransformListener listener;
//     static tf::StampedTransform transform;
//     try{
//         listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
//         listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, transform);
//     } 
//     catch (tf::TransformException ex){
//         // ROS_ERROR("lidar no tf");
//         return;
//     }

//     double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
//     xCur = transform.getOrigin().x();
//     yCur = transform.getOrigin().y();
//     zCur = transform.getOrigin().z();
//     tf::Matrix3x3 m(transform.getRotation());
//     m.getRPY(rollCur, pitchCur, yawCur);
//     Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

//     // 1. convert laser cloud message to pcl
//     pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
//     pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

//     // 2. downsample new cloud (save memory)
//     pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
//     static pcl::VoxelGrid<PointType> downSizeFilter;
//     downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
//     downSizeFilter.setInputCloud(laser_cloud_in);
//     downSizeFilter.filter(*laser_cloud_in_ds);
//     *laser_cloud_in = *laser_cloud_in_ds;

//     // 3. filter lidar points (only keep points in camera view)
//     pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
//     for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
//     {
//         PointType p = laser_cloud_in->points[i];
//         if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
//             laser_cloud_in_filter->push_back(p);
//     }
//     *laser_cloud_in = *laser_cloud_in_filter;

//     // TODO: transform to IMU body frame
//     // 4. offset T_lidar -> T_camera 
//     pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
//     Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
//     pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
//     *laser_cloud_in = *laser_cloud_offset;

//     // 5. transform new cloud into global odom frame
//     pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
//     pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

//     // 6. save new cloud
//     double timeScanCur = laser_msg->header.stamp.toSec();
//     cloudQueue.push_back(*laser_cloud_global);
//     timeQueue.push_back(timeScanCur);

//     // 7. pop old cloud
//     while (!timeQueue.empty())
//     {
//         if (timeScanCur - timeQueue.front() > 5.0)
//         {
//             cloudQueue.pop_front();
//             timeQueue.pop_front();
//         } else {
//             break;
//         }
//     }

//     std::lock_guard<std::mutex> lock(mtx_lidar);
//     // 8. fuse global cloud
//     depthCloud->clear();
//     for (int i = 0; i < (int)cloudQueue.size(); ++i)
//         *depthCloud += cloudQueue[i];

//     // 9. downsample global cloud
//     pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
//     downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
//     downSizeFilter.setInputCloud(depthCloud);
//     downSizeFilter.filter(*depthCloudDS);
//     *depthCloud = *depthCloudDS;
// }


//当前帧5s内的激光点云组成深度点云
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
    std::cout<<"订阅去除畸变的激光点云(雷达坐标系下)"<<std::endl;
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;

    /****************************查询当前帧激光对应的激光里程计 start 20250910***************************************/
    // static tf::TransformListener listener_lio;
    // static tf::StampedTransform transform_lio;
    // try{
    //     listener_lio.waitForTransform("odom", "lidar_link", laser_msg->header.stamp, ros::Duration(0.1));
    //     listener_lio.lookupTransform("odom", "lidar_link", laser_msg->header.stamp, transform_lio);
    // } 
    // catch (tf::TransformException ex){
    //     ROS_ERROR("no tf from odom --> base_link");
    //     return;
    // }
    // double liorollCur, liopitchCur, lioyawCur;
    // tf::Matrix3x3 lio_rotation_matrix(transform_lio.getRotation());
    // lio_rotation_matrix.getRPY(liorollCur, liopitchCur, lioyawCur);
    // Eigen::Affine3f lio_transform = pcl::getTransformation(transform_lio.getOrigin().x(), transform_lio.getOrigin().y(), transform_lio.getOrigin().z(), 
    // liorollCur, liopitchCur, lioyawCur);
    // std::cout<<"激光里程计变换矩阵："<<std::endl;
    /****************************查询当前帧激光对应的激光里程计 end 20250910***************************************/






    // 0. listen to transform
    static tf::TransformListener listener;
    static tf::StampedTransform transform; //这是个视觉定位得到的TF
    try{
        // waitForTransform( [父类坐标系], [子类坐标系], [在这一时刻], [时间段] )
        // 时间段为 waitForTransform() 函数 的结束条件：最多等待 4 秒，如果提前得到了坐标的转换信息，直接结束等待。
        listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
        listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, transform);
    } 
    catch (tf::TransformException ex){
        ROS_ERROR("no TF from vins_world --> vins_body_ros");
        return;
    }

    // std::cout<<"查询视觉TF变换"<<std::endl;
    //把雷达系的点变换到相机系
    //camera_to_ros_transform是将相机系转为标准ROS系，lidar_to_camera_transform是雷达到相机的外参
    tf::Transform trans_lidar_in_world = transform * camera_to_ros_transform.inverse() * lidar_to_camera_transform.inverse();
    tf::Transform transform_ = trans_lidar_in_world;

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform_.getOrigin().x();
    yCur = transform_.getOrigin().y();
    zCur = transform_.getOrigin().z();
    tf::Matrix3x3 m(transform_.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in); //订阅到的去畸变后的激光点云帧

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    // downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    // downSizeFilter.setInputCloud(laser_cloud_in); //
    // downSizeFilter.filter(*laser_cloud_in_ds);
    // *laser_cloud_in = *laser_cloud_in_ds;

    /************************************20250911 add**************************************************/
    // pcl::PointCloud<PointType>::Ptr laser_cloud_1(new pcl::PointCloud<PointType>());
    // pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_1, lio_transform);
    // *laser_cloud_in = *laser_cloud_1;
    /*************************************************************************************************/


    // trans lidar point to ros standard frame，转到ROS标准坐标系下
    tf::Transform ros_to_lidar = camera_to_ros_transform.inverse() * lidar_to_camera_transform.inverse();
    double roll, pitch, yaw;
    tf::Matrix3x3(ros_to_lidar.getRotation()).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transOffset = pcl::getTransformation(ros_to_lidar.getOrigin().getX(), ros_to_lidar.getOrigin().getY(), ros_to_lidar.getOrigin().getZ(), roll, pitch, yaw);
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        PointType p_trans = laser_cloud_offset->points[i];

        //10是一个经验性的宽松阈值，旨在过滤明显无效点（如数值溢出点），而非精确裁剪视场角
        if (p_trans.x >= 0 && abs(p_trans.y / p_trans.x) <= 50 && abs(p_trans.z / p_trans.x) <= 50)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;





    // TODO: transform to IMU body frame
    // add by YJZ
/*     // 4. offset T_lidar -> ros standard frame  
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset; */

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global); //存入点云队列中
    timeQueue.push_back(timeScanCur);
    // lio_transform.push_back(lio_transform);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
            // lio_transform.pop_front();
        } else {
            break;
        }
    }

    //将队列中的多帧点云合并，需要考虑前后两帧之间的位姿变换
    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS; //深度点云

    //测试一下效果，看看乱不乱
    std::cout<<"测试视觉部分订阅的激光点云"<<std::endl;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*depthCloud, tempCloud);
    tempCloud.header.stamp = laser_msg->header.stamp;
    tempCloud.header.frame_id =  laser_msg->header.frame_id;
    pub_test.publish(tempCloud); 

    /************************************20250911 add**************************************************/
    // pcl::PointCloud<PointType>::Ptr laser_cloud_2(new pcl::PointCloud<PointType>());
    // pcl::transformPointCloud(*depthCloudDS, *laser_cloud_2, lio_transform.inverse());
    // *depthCloud = *laser_cloud_2;
    /*************************************************************************************************/

    /************************20250910测试深度点云效果 start************************8*/


    /************************20250910测试深度点云效果 end****************************/
}


//当前帧5s内的激光点云组成深度点云
void lidar_callback2(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
    std::cout<<"订阅去除畸变的激光点云(雷达坐标系下)"<<std::endl;
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;


    // // 0. listen to transform
    // static tf::TransformListener listener;
    // static tf::StampedTransform transform; //这是个视觉定位得到的TF
    // try{
    //     // waitForTransform( [父类坐标系], [子类坐标系], [在这一时刻], [时间段] )
    //     // 时间段为 waitForTransform() 函数 的结束条件：最多等待 4 秒，如果提前得到了坐标的转换信息，直接结束等待。
    //     listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
    //     listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, transform);
    // } 
    // catch (tf::TransformException ex){
    //     ROS_ERROR("no TF from vins_world --> vins_body_ros");
    //     return;
    // }

    // std::cout<<"查询视觉TF变换"<<std::endl;
    //把雷达系的点变换到相机系
    //camera_to_ros_transform是将相机系转为标准ROS系，lidar_to_camera_transform是雷达到相机的外参
    tf::Transform trans_lidar_in_world = lidar_to_camera_transform;
    tf::Transform transform_ = trans_lidar_in_world;

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform_.getOrigin().x();
    yCur = transform_.getOrigin().y();
    zCur = transform_.getOrigin().z();
    tf::Matrix3x3 m(transform_.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in); //订阅到的去畸变后的激光点云帧

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    // downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    // downSizeFilter.setInputCloud(laser_cloud_in); //
    // downSizeFilter.filter(*laser_cloud_in_ds);
    // *laser_cloud_in = *laser_cloud_in_ds;


    // trans lidar point to ros standard frame，转到ROS标准坐标系下
    // tf::Transform ros_to_lidar = lidar_to_camera_transform;
    // double roll, pitch, yaw;
    // tf::Matrix3x3(ros_to_lidar.getRotation()).getRPY(roll, pitch, yaw);
    // Eigen::Affine3f transOffset = pcl::getTransformation(ros_to_lidar.getOrigin().getX(), ros_to_lidar.getOrigin().getY(), ros_to_lidar.getOrigin().getZ(), roll, pitch, yaw);
    // pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    // pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        PointType p_trans = laser_cloud_in->points[i];

        //10是一个经验性的宽松阈值，旨在过滤明显无效点（如数值溢出点），而非精确裁剪视场角
        if (p_trans.x >= 0 && abs(p_trans.y / p_trans.x) <= 50 && abs(p_trans.z / p_trans.x) <= 50)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;



    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global); //存入点云队列中
    timeQueue.push_back(timeScanCur);
    // lio_transform.push_back(lio_transform);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
            // lio_transform.pop_front();
        } else {
            break;
        }
    }

    //将队列中的多帧点云合并，需要考虑前后两帧之间的位姿变换
    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS; //深度点云

    //测试一下效果，看看乱不乱
    std::cout<<"测试视觉部分订阅的激光点云"<<std::endl;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*depthCloud, tempCloud);
    tempCloud.header.stamp = laser_msg->header.stamp;
    tempCloud.header.frame_id =  laser_msg->header.frame_id;
    pub_test.publish(tempCloud); 

}

// //当前帧5s内的激光点云组成深度点云
// void lidar_callback1(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
// {
//     std::cout<<"订阅去除畸变的激光点云"<<std::endl;
//     static int lidar_count = -1;
//     if (++lidar_count % (LIDAR_SKIP+1) != 0)
//         return;



//     // 0. listen to transform
//     static tf::TransformListener listener;
//     static tf::StampedTransform transform;
//     try{
//         listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
//         listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, transform);
//     } 
//     catch (tf::TransformException ex){
//         // ROS_ERROR("lidar no tf");
//         return;
//     }

//     //camera_to_ros_transform是将相机系转为标准ROS系，lidar_to_camera_transform是雷达到相机的外参
//     tf::Transform trans_lidar_in_world = transform * camera_to_ros_transform.inverse() * lidar_to_camera_transform.inverse();
//     tf::Transform transform_ = trans_lidar_in_world;

//     double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
//     xCur = transform_.getOrigin().x();
//     yCur = transform_.getOrigin().y();
//     zCur = transform_.getOrigin().z();
//     tf::Matrix3x3 m(transform_.getRotation());
//     m.getRPY(rollCur, pitchCur, yawCur);
//     Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

//     // 1. convert laser cloud message to pcl
//     pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
//     pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

//     // 2. downsample new cloud (save memory)
//     pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
//     static pcl::VoxelGrid<PointType> downSizeFilter;
//     downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
//     downSizeFilter.setInputCloud(laser_cloud_in);
//     downSizeFilter.filter(*laser_cloud_in_ds);
//     *laser_cloud_in = *laser_cloud_in_ds;

//     /************************************20250911 add**************************************************/
//     // pcl::PointCloud<PointType>::Ptr laser_cloud_1(new pcl::PointCloud<PointType>());
//     // pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_1, lio_transform);
//     // *laser_cloud_in = *laser_cloud_1;
//     /*************************************************************************************************/


//     // // trans lidar point to ros standard frame
//     // tf::Transform ros_to_lidar = camera_to_ros_transform.inverse() * lidar_to_camera_transform.inverse();
//     // double roll, pitch, yaw;
//     // tf::Matrix3x3(ros_to_lidar.getRotation()).getRPY(roll, pitch, yaw);
//     // Eigen::Affine3f transOffset = pcl::getTransformation(ros_to_lidar.getOrigin().getX(), ros_to_lidar.getOrigin().getY(), ros_to_lidar.getOrigin().getZ(), roll, pitch, yaw);
//     // pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
//     // pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);

//     // // 3. filter lidar points (only keep points in camera view)
//     // pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
//     // for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
//     // {
//     //     PointType p = laser_cloud_in->points[i];
//     //     PointType p_trans = laser_cloud_offset->points[i];

//     //     //10是一个经验性的宽松阈值，旨在过滤明显无效点（如数值溢出点），而非精确裁剪视场角
//     //     if (p_trans.x >= 0 && abs(p_trans.y / p_trans.x) <= 10 && abs(p_trans.z / p_trans.x) <= 10)
//     //         laser_cloud_in_filter->push_back(p);
//     // }
//     // *laser_cloud_in = *laser_cloud_in_filter;



//     // // 5. transform new cloud into global odom frame
//     // pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
//     // pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

//     // 6. save new cloud
//     double timeScanCur = laser_msg->header.stamp.toSec();
//     cloudQueue.push_back(*laser_cloud_global);
//     timeQueue.push_back(timeScanCur);
//     lio_transform_deque.push_back(lio_transform);
//     transform_offset_deque.push_back(transOffset);
//     transform_now_deque.push_back(transNow);

//     // 7. pop old cloud
//     while (!timeQueue.empty())
//     {
//         if (timeScanCur - timeQueue.front() > 5.0)
//         {
//             cloudQueue.pop_front();
//             timeQueue.pop_front();
//             lio_transform_deque.pop_front();
//             transform_offset_deque.pop_front();
//             transform_now_deque.pop_front();
//         } else {
//             break;
//         }
//     }

//     //先将队列中的点变换到最后一帧上
//     for(int i = (int)cloudQueue.size()-2; i >=0; --i)
//     {
//         pcl::PointCloud<PointType>::Ptr laser_cloud_i(new pcl::PointCloud<PointType>());
//         Eigen::Affine3f transBetween = lio_transform_deque[i].inverse()*lio_transform_deque[i+1];
//         pcl::transformPointCloud(cloudQueue[i], *laser_cloud_offset, transBetween);
//     }





//     //将队列中的多帧点云合并，需要考虑前后两帧之间的位姿变换
//     std::lock_guard<std::mutex> lock(mtx_lidar);
//     // 8. fuse global cloud
//     depthCloud->clear();
//     for (int i = 0; i < (int)cloudQueue.size(); ++i)
//     {
//         // *depthCloud += cloudQueue[i];




//         //0.1 先进行一次ofset变换
//         pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
//         pcl::transformPointCloud(cloudQueue[i], *laser_cloud_offset, transform_offset_deque[i]);


//     }
        

//     // 9. downsample global cloud
//     pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
//     downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
//     downSizeFilter.setInputCloud(depthCloud);
//     downSizeFilter.filter(*depthCloudDS);
//     *depthCloud = *depthCloudDS; //深度点云

//     /************************************20250911 add**************************************************/
//     // pcl::PointCloud<PointType>::Ptr laser_cloud_2(new pcl::PointCloud<PointType>());
//     // pcl::transformPointCloud(*depthCloudDS, *laser_cloud_2, lio_transform.inverse());
//     // *depthCloud = *laser_cloud_2;
//     /*************************************************************************************************/

//     /************************20250910测试深度点云效果 start************************8*/


//     /************************20250910测试深度点云效果 end****************************/
// }



int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    readParameters(n);

    // read camera params
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        std::cout<<"使用相机的数量："<<NUM_OF_CAM<<std::endl;
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);
    }
        

    // load fisheye mask to remove features on the boundry
    std::cout<< ((FISHEYE > 0)?"是":"未")<<"使用鱼眼相机"<<std::endl;
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_ERROR("load fisheye mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // initialize depthRegister (after readParameters())
    depthRegister = new DepthRegister(n);
    
    // subscriber to image and lidar
    std::cout<<"相机话题："<<IMAGE_TOPIC<<std::endl;
    ros::Subscriber sub_img   = n.subscribe(IMAGE_TOPIC,       5,    imgCompressed_callback);
    ros::Subscriber sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 5,    lidar_callback);
    if (!USE_LIDAR)
        sub_lidar.shutdown();

    // messages to vins estimator
    pub_feature = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5); //视觉特征点话题
    pub_match   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5); //用于可视化
    pub_restart = n.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);
    pub_test   = n.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/test",     5); //用于测试

    // two ROS spinners for parallel processing (image and lidar)
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
