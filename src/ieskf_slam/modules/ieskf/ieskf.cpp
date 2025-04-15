/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-13 16:43:29
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 16:01:59
 */
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/math/mean_cov_filter.h"

namespace IESKFSlam {

    IESKF::IESKF(const std::string &config_path, const std::string &prefix)
        : ModuleBase(config_path, prefix, "IESKF") {
        P.setIdentity();
        P(9, 9) = P(10, 10) = P(11, 11) = 0.0001;
        P(12, 12) = P(13, 13) = P(14, 14) = 0.001;
        P(15, 15) = P(16, 16) = P(17, 17) = 0.00001;
        double cov_gyroscope, cov_acceleration, cov_bias_acceleration, cov_bias_gyroscope;
        readParam("cov_gyroscope", cov_gyroscope, 0.1);
        readParam("cov_acceleration", cov_acceleration, 0.1);
        readParam("cov_bias_acceleration", cov_bias_acceleration, 0.1);
        readParam("cov_bias_gyroscope", cov_bias_gyroscope, 0.1);
        Q.block<3, 3>(0, 0).diagonal() =
            Eigen::Vector3d{cov_gyroscope, cov_gyroscope, cov_gyroscope};
        Q.block<3, 3>(3, 3).diagonal() =
            Eigen::Vector3d{cov_acceleration, cov_acceleration, cov_acceleration};
        Q.block<3, 3>(6, 6).diagonal() =
            Eigen::Vector3d{cov_bias_gyroscope, cov_bias_gyroscope, cov_bias_gyroscope};
        Q.block<3, 3>(9, 9).diagonal() =
            Eigen::Vector3d{cov_bias_acceleration, cov_bias_acceleration, cov_bias_acceleration};
        X.ba.setZero();
        X.bg.setZero();
        X.gravity.setZero();
        X.position.setZero();
        X.rotation.setIdentity();
        X.velocity.setZero();
        print_table();
    }

    IESKF::~IESKF() {}

    void IESKF::predict(IMU imu, double dt) {

        static Eigen::Matrix<double, 18, 18> Fx;
        static Eigen::Matrix<double, 18, 12> Fw;
        static Eigen::Matrix3d rotation, Idt,  A_mat, Rdt;
        static Eigen::Vector3d gyr_dt;

        Idt = I3 * dt;

        imu.acceleration -= X.ba;
        imu.gyroscope -= X.bg;
        rotation = X.rotation.toRotationMatrix();
        
        gyr_dt = imu.gyroscope * dt;
        A_mat = A_T(-gyr_dt);
        Rdt = rotation * dt;

        X.rotation =
            Eigen::Quaterniond(X.rotation.toRotationMatrix() * so3Exp(gyr_dt));
        X.rotation.normalize();
        X.position += X.velocity * dt;
        X.velocity += (rotation * imu.acceleration + X.gravity) * dt;
        Fw.setZero();
        Fx.setIdentity();
        Fx.block<3, 3>(0, 0) = so3Exp(-1 * gyr_dt);

        Fx.block<3, 3>(0, 9) = -1 * A_mat * dt;

        Fx.block<3, 3>(3, 6) = Idt;
        Fx.block<3, 3>(6, 0) = -Rdt * skewSymmetric(imu.acceleration) ;
        Fx.block<3, 3>(6, 12) = -Rdt;
        Fx.block<3, 3>(6, 15) = Idt;
        Fw.block<3, 3>(0, 0) = -1 * A_mat * dt;
        Fw.block<3, 3>(6, 3) = -Rdt;
        Fw.block<3, 3>(9, 6) = Fw.block<3, 3>(12, 9) = Idt;
        P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose();
    }

    bool IESKF::update() {

        static int cnt_ = 0;
        
        ///. 开迭
        static Eigen::MatrixXd K;
        static Eigen::MatrixXd H_k;
        static Eigen::Matrix<double, 18, 18> P_in_update;
        static Eigen::Matrix<double, 18, 1> error_state ;
        static Eigen::Matrix<double, 18, 18> J_inv;

        static Eigen::MatrixXd z_k;
        static Eigen::MatrixXd R_inv;
        static Eigen::MatrixXd H_kt;
        static Eigen::MatrixXd update_x;
        static State18 x_k_k;

        x_k_k =  X;

        bool converge = true;
        for (int i = 0; i < iter_times; i++) {
            ///. 计算误差状态 J
            error_state = getErrorState18(x_k_k, X);
            
            J_inv.setIdentity();
            J_inv.block<3, 3>(0, 0) = A_T(error_state.block<3, 1>(0, 0));
            // 更新 P
            P_in_update = J_inv * P * J_inv.transpose();
            
            // 调用接口计算 Z H
            calc_zh_ptr->calculate(x_k_k, z_k, H_k);
            H_kt = H_k.transpose();
            // R 直接写死0.001;
            K = (H_kt * H_k + (P_in_update / 0.001).inverse()).inverse() * H_kt;
            //. 计算X 的增量

            update_x = -1 * K * z_k  -  (I18 - K * H_k) * J_inv * error_state;

            // 收敛判断
            converge = true;
            for (int idx = 0; idx < 18; idx++) {
                if (update_x(idx, 0) > 0.001) {
                    converge = false;
                    break;
                }
            }
            // 更新X
            x_k_k.rotation = x_k_k.rotation.toRotationMatrix() * so3Exp(update_x.block<3, 1>(0, 0));
            x_k_k.rotation.normalize();
            x_k_k.position = x_k_k.position + update_x.block<3, 1>(3, 0);
            x_k_k.velocity = x_k_k.velocity + update_x.block<3, 1>(6, 0);
            x_k_k.bg = x_k_k.bg + update_x.block<3, 1>(9, 0);
            x_k_k.ba = x_k_k.ba + update_x.block<3, 1>(12, 0);
            x_k_k.gravity = x_k_k.gravity + update_x.block<3, 1>(15, 0);
            if (converge) {
                break;
            }
        }
        cnt_++;
        X = x_k_k;
        P = (I18 - K * H_k) * P_in_update;
        
        if(converge)
            return true;

        for (int idx = 0; idx < 18; idx++) {
            if (update_x(idx, 0) > 0.1) {
                return false;
            }
        }

        return true;
    }
    
    Eigen::Matrix<double, 18, 1> IESKF::getErrorState18(const State18 &s1, const State18 &s2) {
        static Eigen::Matrix<double, 18, 1> es;
        es.setZero();
        es.block<3, 1>(0, 0) =
            SO3Log(s2.rotation.toRotationMatrix().transpose() * s1.rotation.toRotationMatrix());
        es.block<3, 1>(3, 0) = s1.position - s2.position;
        es.block<3, 1>(6, 0) = s1.velocity - s2.velocity;
        es.block<3, 1>(9, 0) = s1.bg - s2.bg;
        es.block<3, 1>(12, 0) = s1.ba - s2.ba;
        es.block<3, 1>(15, 0) = s1.gravity - s2.gravity;
        return es;
    }
    const IESKF::State18 &IESKF::getX() { return X; }
    void IESKF::setX(const IESKF::State18 &x_in) { X = x_in; }
}  // namespace IESKFSlam
