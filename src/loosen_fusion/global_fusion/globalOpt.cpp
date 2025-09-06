/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "globalOpt.h"
#include "Factors.h"

double ori_lat = 49.049500;
double ori_lon = 8.396601;
double ori_alt = 113.057571;
// double GlobalOptimization::ori_lat = 0;
// double GlobalOptimization::ori_lon = 0;
// double GlobalOptimization::ori_alt = 0;


GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
    newGPS = false;
	WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    WGPS_T_WLIO = Eigen::Matrix4d::Identity();

    std::cout<<"11:"<<ori_lat<<","<<ori_lon<<std::endl;
    setOrigin(ori_lat, ori_lon, ori_alt);

    threadOpt = std::thread(&GlobalOptimization::optimize, this); //另开一个线程执行
}



void GlobalOptimization::setOrigin(double lat, double lon, double alt)
{
    init_gps_longitude = lon;
    init_gps_latitude = lat;
    init_gps_altitude = alt;
    geoConverter.Reset(lat, lon, alt);
    initGPS = true;
    ROS_INFO("fusion node ENU origin set to: lat=%.8f, lon=%.8f, alt=%.3f", lat, lon, alt);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

//将GPS经纬高转为xyz
void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
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

//输入视觉里程计
void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;//视觉局部位姿


    Eigen::Quaterniond globalQ;
    //将视觉里程计转换到gps坐标系下
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose; //视觉全局位姿
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped); //全局坐标系下的视觉轨迹

    mPoseMap.unlock();
}

//输入激光里程计
void GlobalOptimization::inputLidarOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap1.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localLidarPoseMap[t] = localPose;//激光局部位姿


    Eigen::Quaterniond globalQ;
    //将激光里程计转换到gps坐标系下
    globalQ = WGPS_T_WLIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WLIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WLIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalLIOPoseMap[t] = globalPose; //激光全局位姿
    LIO_lastP = globalP;
    LIO_lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = LIO_lastP.x();
    pose_stamped.pose.position.y = LIO_lastP.y();
    pose_stamped.pose.position.z = LIO_lastP.z();
    pose_stamped.pose.orientation.x = LIO_lastQ.x();
    pose_stamped.pose.orientation.y = LIO_lastQ.y();
    pose_stamped.pose.orientation.z = LIO_lastQ.z();
    pose_stamped.pose.orientation.w = LIO_lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped); //全局坐标系下的视觉轨迹

    mPoseMap1.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    //printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	GPSPositionMap[t] = tmp;
    newGPS = true;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = xyz[0];
    pose_stamped.pose.position.y = xyz[1];
    pose_stamped.pose.position.z = xyz[2];
    pose_stamped.pose.orientation.w = 1;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    gnss_path.poses.push_back(pose_stamped);

}

void GlobalOptimization::inputGPSOdom(double t, nav_msgs::OdometryConstPtr GPS_msg)
{
	vector<double> tmp{GPS_msg->pose.pose.position.x, GPS_msg->pose.pose.position.y, GPS_msg->pose.pose.position.z};
    //printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	GPSPositionMap[t] = tmp;
    newGPS = true;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = GPS_msg->pose.pose.position.x;
    pose_stamped.pose.position.y = GPS_msg->pose.pose.position.y;
    pose_stamped.pose.position.z = GPS_msg->pose.pose.position.z;
    pose_stamped.pose.orientation.w = 1;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    gnss_path.poses.push_back(pose_stamped);

}


//执行优化
void GlobalOptimization::optimize()
{
    while(true)
    {
        //如果有新的GPS数据
        if(newGPS)
        {
            newGPS = false;
            // printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            //添加参数块
            //添加视觉局部位姿参数块
            mPoseMap.lock();

            // if(localPoseMap.size() > 0)
            // {
            int length = localPoseMap.size(); 
            // w^t_i   w^q_i
            double t_array[length][3];//平移
            double q_array[length][4];//旋转
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++) //这是以视觉里程计为基础，对视觉里程计进行优化
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }
            // }


            // //添加激光局部位姿参数块
            // int lio_length = localLidarPoseMap.size(); 
            // // w^t_i   w^q_i
            // double lio_t_array[lio_length][3];//平移
            // double lio_q_array[lio_length][4];//旋转
            // map<double, vector<double>>::iterator lio_iter;
            // lio_iter = globalLIOPoseMap.begin();
            // for (int i = 0; i < lio_length; i++, iter++) //这是以激光里程计为基础，对激光里程计进行优化
            // {
            //     lio_t_array[i][0] = lio_iter->second[0];
            //     lio_t_array[i][1] = lio_iter->second[1];
            //     lio_t_array[i][2] = lio_iter->second[2];
            //     lio_q_array[i][0] = lio_iter->second[3];
            //     lio_q_array[i][1] = lio_iter->second[4];
            //     lio_q_array[i][2] = lio_iter->second[5];
            //     lio_q_array[i][3] = lio_iter->second[6];
            //     problem.AddParameterBlock(lio_q_array[i], 4, local_parameterization);
            //     problem.AddParameterBlock(lio_t_array[i], 3);
            // }




            //添加视觉残差块
            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;//相邻两帧之间的变换矩阵
                    
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                    /*
                    double **para = new double *[4];
                    para[0] = q_array[i];
                    para[1] = t_array[i];
                    para[3] = q_array[i+1];
                    para[4] = t_array[i+1];

                    double *tmp_r = new double[6];
                    double **jaco = new double *[4];
                    jaco[0] = new double[6 * 4];
                    jaco[1] = new double[6 * 3];
                    jaco[2] = new double[6 * 4];
                    jaco[3] = new double[6 * 3];
                    vio_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                        << std::endl;
                    */
                }



                //GPS因子

                //gps factor
                double t = iterVIO->first;
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], iterGPS->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                    /*
                    double **para = new double *[1];
                    para[0] = t_array[i];

                    double *tmp_r = new double[3];
                    double **jaco = new double *[1];
                    jaco[0] = new double[3 * 3];
                    gps_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    */
                }
            }


            // //添加激光残差块
            // map<double, vector<double>>::iterator iterLIO, iterLIONext, iterGPSL;
            // int j = 0;
            // for (iterLIO = localLidarPoseMap.begin(); iterLIO != localLidarPoseMap.end(); iterLIO++, j++)
            // {
            //     //lio factor
            //     iterLIONext = iterLIO;
            //     iterLIONext++;
            //     if(iterLIONext != localLidarPoseMap.end())
            //     {
            //         Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
            //         Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
            //         wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterLIO->second[3], iterLIO->second[4], 
            //                                                    iterLIO->second[5], iterLIO->second[6]).toRotationMatrix();
            //         wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterLIO->second[0], iterLIO->second[1], iterLIO->second[2]);
            //         wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterLIONext->second[3], iterLIONext->second[4], 
            //                                                    iterLIONext->second[5], iterLIONext->second[6]).toRotationMatrix();
            //         wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterLIONext->second[0], iterLIONext->second[1], iterLIONext->second[2]);
            //         Eigen::Matrix4d iTj = wTi.inverse() * wTj;//相邻两帧之间的变换矩阵
                    
            //         Eigen::Quaterniond iQj;
            //         iQj = iTj.block<3, 3>(0, 0);
            //         Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

            //         ceres::CostFunction* lio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
            //                                                                     iQj.w(), iQj.x(), iQj.y(), iQj.z(),
            //                                                                     0.1, 0.01);
            //         problem.AddResidualBlock(lio_function, NULL, lio_q_array[i], lio_t_array[i], lio_q_array[i+1], lio_t_array[i+1]);

            //         /*
            //         double **para = new double *[4];
            //         para[0] = q_array[i];
            //         para[1] = t_array[i];
            //         para[3] = q_array[i+1];
            //         para[4] = t_array[i+1];

            //         double *tmp_r = new double[6];
            //         double **jaco = new double *[4];
            //         jaco[0] = new double[6 * 4];
            //         jaco[1] = new double[6 * 3];
            //         jaco[2] = new double[6 * 4];
            //         jaco[3] = new double[6 * 3];
            //         vio_function->Evaluate(para, tmp_r, jaco);

            //         std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
            //             << std::endl;
            //         std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
            //             << std::endl;
            //         std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
            //             << std::endl;
            //         std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
            //             << std::endl;
            //         std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
            //             << std::endl;
            //         */
            //     }



            //     //GPS因子

            //     //gps factor
            //     double t = iterLIO->first;
            //     iterGPS = GPSPositionMap.find(t);
            //     if (iterGPS != GPSPositionMap.end())
            //     {
            //         ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
            //                                                            iterGPS->second[2], iterGPS->second[3]);
            //         //printf("inverse weight %f \n", iterGPS->second[3]);
            //         problem.AddResidualBlock(gps_function, loss_function, lio_t_array[i]);

            //         /*
            //         double **para = new double *[1];
            //         para[0] = t_array[i];

            //         double *tmp_r = new double[3];
            //         double **jaco = new double *[1];
            //         jaco[0] = new double[3 * 3];
            //         gps_function->Evaluate(para, tmp_r, jaco);

            //         std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
            //             << std::endl;
            //         std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
            //             << std::endl;
            //         */
            //     }
            // }


            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            //更新全局位姿
            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    } //end while
	return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}