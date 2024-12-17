#pragma once

#include <ArduinoEigen.h>

#include <map>
#include <vector>

#include "Control/Dynamics/pose_transform.hpp"
#include "Control/Dynamics/rigid_transform.hpp"

class Kinematics {
private:
    /* data */
    std::map<int, RigidTransform> htmat;
    int joint_num_;
    int tool_base_num_;
    int tool_point_num_;

    Eigen::Vector3d tool_pos;
    Eigen::Matrix3d tool_rot;

    PoseTransform pose_transform_;
    void init() {};

public:
    Kinematics(int joint_num)
        : joint_num_(joint_num),
          tool_base_num_(joint_num + 1),
          tool_point_num_(joint_num + 2) {};
    ~Kinematics() {};
    void set_rt_param(int joint_num, Eigen::Vector3d p, Eigen::Matrix3d R,
                      double theta) {
        htmat[joint_num].set_dhparam(p, R, theta);
    };
    void set_rt_tool_pose(Eigen::Vector3d p, Eigen::Matrix3d R) {
        tool_pos = p;
        tool_rot = R;
        Eigen::Matrix4d temp;
        htmat[tool_point_num_ - 1].set_dhparam(p, R, 0.0);
    };
    //
    void calc_joint_htmat(Eigen::VectorXd q,
                          std::map<int, Eigen::Matrix4d>& htmat_out) {
        Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
        for (int i = 0; i < tool_point_num_ - 1; i++) {
            temp *= htmat[i].get_htmat(q[i]);
            htmat_out[i] = temp;
        }
    };
    void calc_basic_jacobian_matrix(Eigen::VectorXd q,
                                    Eigen::MatrixXd& jacobian_matrix) {
        Eigen::VectorXd q_ = Eigen::VectorXd::Zero(tool_point_num_);
        for (int i = 0; i < joint_num_; i++) {
            q_[i] = q[i];
        }
        // temp_p_i and temp_p_i
        std::vector<Eigen::Vector3d> p_i;
        std::vector<Eigen::Matrix3d> R_i;
        p_i.reserve(tool_base_num_);
        R_i.reserve(tool_base_num_);
        Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
        for (int i = 0; i < tool_base_num_; i++) {
            temp = htmat[i].get_htmat(q_[i]);
            p_i.push_back(temp.block<3, 1>(0, 3));
            R_i.push_back(temp.block<3, 3>(0, 0));
        }
        // R_i -> R_i_0
        std::vector<Eigen::Vector3d> R_i_0;
        R_i_0.reserve(joint_num_);
        Eigen::Matrix3d temp_R = Eigen::Matrix3d::Identity();
        for (int i = 0; i < tool_base_num_; i++) {
            temp_R *= R_i[i];
            R_i_0.push_back(temp_R);
        }
        // R_i_0, p_i -> rp
        std::vector<Eigen::Vector3d> rp;
        rp.reserve(joint_num_);
        for (int i = 0; i < joint_num_; i++) {
            rp.push_back(R_i_0[i] * p_i[i + 1]);
        }
        // rp -> pe
        std::vector<Eigen::Vector3d> pe;
        pe.reserve(joint_num_);
        for (int i = 0; i < joint_num_; i++) {
            pe.push_back(Eigen::Vector3d::Zero());
            for (int j = 0; j < joint_num_ - i; j++) {
                pe[i] += rp[i + j];
            }
        }
        // R_i_0 -> z
        std::vector<Eigen::Vector3d> z;
        z.reserve(joint_num_);
        for (int i = 0; i < joint_num_; i++) {
            z.push_back(R_i_0[i] * Eigen::Vector3d::UnitZ());
        }
        // z, pe -> zp
        std::vector<Eigen::Vector3d> zp;
        zp.reserve(joint_num_);
        for (int i = 0; i < joint_num_; i++) {
            zp.push_back(z[i].cross(pe[i]));
        }
        // jacobi matrix
        for (int i = 0; i < joint_num_; i++) {
            jacobian_matrix.block<3, 1>(0, i) = zp[i];
            jacobian_matrix.block<3, 1>(3, i) = z[i];
        }
    };
    //
    void calc_time_derivative_of_basic_jacobian_matrix(
        Eigen::VectorXd q, Eigen::MatrixXd& jacobian_matrix) {

    };

    void calc_forward_kinematics(Eigen::VectorXd q, Eigen::Vector3d& p,
                                 Eigen::Matrix3d& R) {
        Eigen::VectorXd q_ = Eigen::VectorXd::Zero(tool_point_num_);
        for (int i = 0; i < joint_num_; i++) {
            q_[i] = q[i];
        }
        q_[joint_num_] = 0.0;
        //
        Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
        for (int i = 0; i < tool_point_num_ - 1; i++) {
            temp *= htmat[i].get_htmat(q[i]);
        }
        p = temp.block<3, 1>(0, 3);
        R = temp.block<3, 3>(0, 0);
    };

    void calc_error_pose(Eigen::Matrix4d src, Eigen::Matrix4d dst,
                         Eigen::MatrixXd& error_pose) {
        Eigen::Vector3d p, pd;
        Eigen::Matrix3d R, Rd;
        pose_transform_.transform_htmat2pos_rotmat(dst, pd, Rd);
        pose_transform_.transform_htmat2pos_rotmat(src, p, R);
        //
        Eigen::VectorXd err(6);
        // err pos
        err.block<3, 1>(0, 0) = p - pd;
        // err rot
        Eigen::Matrix3d err_rot = R.transpose() * Rd;

        if (err_rot(0, 0) * err_rot(1, 1) * err_rot(2, 2) > 0.999999) {
            if (err_rot.trace() > 2.2999999) {
                err.block<3, 1>(3, 0) = Eigen::Vector3d::Zero();
            } else {
                err[3] = -PI * 0.5 * (err_rot(0, 0) - 1.0);
                err[4] = -PI * 0.5 * (err_rot(1, 1) - 1.0);
                err[5] = -PI * 0.5 * (err_rot(2, 2) - 1.0);
            }
        } else {
            Eigen::Vector3d err_l;
            Eigen::Vector3d rot_err;
            err_l[0] = err_rot(2, 1) - err_rot(1, 2);
            err_l[1] = err_rot(0, 2) - err_rot(2, 0);
            err_l[2] = err_rot(1, 0) - err_rot(0, 1);
            double err_l_norm = err_l.norm();
            rot_err = std::atan2(err_l_norm, err_rot.trace() - 1) * err_l /
                      err_l_norm;
            err.block<3, 1>(3, 0) = -rot_err;
        }
    };
};