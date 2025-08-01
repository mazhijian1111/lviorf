#include "imu_processor.h"

#include <glog/logging.h>
#include <Eigen/Dense>

#include "utils.h"

namespace ImuGpsLocalization {

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

ImuProcessor::ImuProcessor(const double acc_noise, const double gyro_noise,
                           const double acc_bias_noise, const double gyro_bias_noise,
                           const Eigen::Vector3d& gravity)
    : acc_noise_(acc_noise), gyro_noise_(gyro_noise), 
      acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
      gravity_(gravity) { }

//IMU预测
void ImuProcessor::Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state) {
    //与上一帧IMU数据的时间差
    const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    const double delta_t2 = delta_t * delta_t;

    //上一帧的状态
    State last_state = *state;

    /***************************2050729:注释掉******************************************* */
    // Acc and gyro.
    //无偏角速度和加速度
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + cur_imu->acc) - last_state.acc_bias;
    const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + cur_imu->gyro) - last_state.gyro_bias;

    // Normal state. 
    // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".
    state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 
                   0.5 * (last_state.G_R_I * acc_unbias + gravity_) * delta_t2;
    state->G_v_I = last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t;

    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        state->G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
    }
    // Error-state. Not needed.

    /****************************2050729:中值积分***************************** */
    // Eigen::Vector3d un_acc_0 = state->G_R_I * (last_imu->acc - last_state.acc_bias) + gravity_;
    // Eigen::Vector3d un_gyr = 0.5 * (last_imu->gyro + cur_imu->gyro) - last_state.gyro_bias;
    // state->G_R_I = last_state.G_R_I*(deltaQ(un_gyr * delta_t).toRotationMatrix());
    // Eigen::Vector3d un_acc_1 = state->G_R_I * (cur_imu->acc - last_state.acc_bias) + gravity_;
    // Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    // state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 0.5 * un_acc * delta_t2;
    // state->G_v_I = last_state.G_v_I + un_acc * delta_t;

    // Eigen::Vector3d acc_unbias = un_acc;
    // Eigen::Vector3d gyro_unbias = un_gyr;
    // Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;

    /********************************************************* */

    // Covariance of the error-state.   
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6)   = - state->G_R_I * GetSkewMatrix(acc_unbias) * delta_t;
    Fx.block<3, 3>(3, 9)   = - state->G_R_I * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
    } else {
        Fx.block<3, 3>(6, 6).setIdentity();
    }
    Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

    state->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Time and imu.
    state->timestamp = cur_imu->timestamp;
    state->imu_data_ptr = cur_imu;
}

}  // namespace ImuGpsLocalization