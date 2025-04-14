/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-18 20:00:47
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:18:14
 */
#pragma once 
#ifndef MATH_SO3
#define MATH_SO3
#include <Eigen/Dense>
#include <sophus/so3.hpp>

namespace IESKFSlam
{

    static Eigen::Matrix3d I3{ Eigen::Matrix3d::Identity() };
    static Eigen::Matrix<double, 4, 4> I4{ Eigen::Matrix<double, 4, 4>::Identity()};
    static Eigen::Matrix<double, 6, 6> I6{ Eigen::Matrix<double, 6, 6>::Identity()};
    static Eigen::Matrix<double, 18, 18> I18{ Eigen::Matrix<double, 18, 18>::Identity()};

    static inline Eigen::Matrix3d  skewSymmetric(const Eigen::Vector3d &so3) {
        return Sophus::SO3d::hat(so3);
    }

    static Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3 ){
        return Sophus::SO3d::exp(so3).matrix();
    }

    static Eigen::Vector3d SO3Log(const Eigen::Matrix3d&SO3 ){
        return Sophus::SO3d(SO3).log();
    } 

    static Eigen::Matrix3d A_T(const Eigen::Vector3d& u)
    {
        double un = u.norm();
        if (un < 1e-4) {
            return I3;
        }
        double inv_sqr_norm = 1.0 / (un * un);

        Eigen::Matrix3d ux = Sophus::SO3d::hat(u);
        return I3 +
           (1.0 - std::cos(un)) * ux * inv_sqr_norm +
           (1.0 - std::sin(un) / un) * ux * ux * inv_sqr_norm;
    }

} // namespace SO3
#endif