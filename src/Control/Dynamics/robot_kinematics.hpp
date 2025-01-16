#pragma once

#include <ArduinoEigen.h>

#include <map>
#include <vector>

#include "Control/Dynamics/pose_transform.hpp"
#include "Control/Dynamics/rigid_transform.hpp"

class RobotKinematics {
private:
    /* data */
    int joint_num_;
    int tool_point_num_;
    //
    Eigen::VectorXd q_;   // J1, ..., Jn, 0 (tool point num = joint num + 1)
    Eigen::VectorXd dq_;  // J1, ..., Jn, 0 (tool point num = joint num + 1)
    // ht_mat R 4x4 tool point num
    std::map<int, RigidTransform> htmat;  // {i}^ T_i+1, i = 0, 1, ..., n
    std::vector<Eigen::Matrix4d> T_from_prev_axis;    // {i}^T_i+1
    std::vector<Eigen::Matrix3d> R_from_prev_axis;    // {i}^R_i+1
    std::vector<Eigen::Vector3d> p_from_prev_axis;    // {i}^p_i,i+1
    std::vector<Eigen::Matrix4d> T_from_origin_axis;  // {0}^T_i+1
    std::vector<Eigen::Matrix3d> R_from_origin_axis;  // {0}^R_i+1
    std::vector<Eigen::Vector3d> p_from_origin_axis;  // {0}^p_i+1
    //
    std::vector<Eigen::Vector3d> z_from_origin_axis;   // {0}^z_i
    std::vector<Eigen::Vector3d> pe_from_origin_axis;  // {0}^pe_i
    std::vector<Eigen::Vector3d> rp_;  // {0}^R_i+1 * {i+1}^p_i+1, i+2
    //
    std::vector<Eigen::Matrix4d> dT_from_prev_axis;  // {i}^dT_i+1
    std::vector<Eigen::Matrix3d> dR_from_prev_axis;  // {i}^dR_i+1
    std::vector<Eigen::Vector3d> dp_from_prev_axis;  // {i}^dp_i,i+1
    //
    // std::vector<Eigen::Vector3d> dpe_from_origin_axis;  // {0}^dpe_i
    std::vector<Eigen::Matrix3d> RdR_from_origin_axis;  // {0}^R_i{i}^dR_i+1
    std::vector<Eigen::Vector3d> dz_;                   // {0}^dz_i
    std::vector<Eigen::Vector3d>
        temp_dpe_;  // {0}^dpe_iの右辺  i = 0, 1, ..., n-1; n = joint num
    std::vector<Eigen::Vector3d> dpe_;
    //
    Eigen::Vector3d tool_pos;
    Eigen::Matrix3d tool_rot;

    PoseTransform pose_transform_;

    // >> Function
    void calc_p_and_R_from_prev_axis();
    void calc_p_and_R_from_origin_axis();
    void calc_z_from_origin_axis();
    void calc_pe_from_origin_axis();
    void calc_dp_and_dR_from_prev_axis();
    void calc_RdR_from_origin_axis();

public:
    RobotKinematics(int joint_num)
        : joint_num_(joint_num), tool_point_num_(joint_num + 1) {
        q_ = Eigen::VectorXd::Zero(tool_point_num_);
        dq_ = Eigen::VectorXd::Zero(tool_point_num_);
        //
        T_from_prev_axis.reserve(tool_point_num_);
        R_from_prev_axis.reserve(tool_point_num_);
        p_from_prev_axis.reserve(tool_point_num_);
        T_from_origin_axis.reserve(tool_point_num_);
        R_from_origin_axis.reserve(tool_point_num_);
        p_from_origin_axis.reserve(tool_point_num_);
        //
        z_from_origin_axis.reserve(tool_point_num_);
        pe_from_origin_axis.reserve(tool_point_num_);
        rp_.reserve(tool_point_num_);
        //
        dT_from_prev_axis.reserve(tool_point_num_);
        dR_from_prev_axis.reserve(tool_point_num_);
        dp_from_prev_axis.reserve(tool_point_num_);
        //
        for (int i = 0; i < tool_point_num_; i++) {
            htmat[i] = RigidTransform();
            //
            T_from_prev_axis.push_back(Eigen::Matrix4d::Identity());
            R_from_prev_axis.push_back(Eigen::Matrix3d::Identity());
            p_from_prev_axis.push_back(Eigen::Vector3d::Zero());
            T_from_origin_axis.push_back(Eigen::Matrix4d::Identity());
            R_from_origin_axis.push_back(Eigen::Matrix3d::Identity());
            p_from_origin_axis.push_back(Eigen::Vector3d::Zero());
            //
            z_from_origin_axis.push_back(Eigen::Vector3d::Zero());
            pe_from_origin_axis.push_back(Eigen::Vector3d::Zero());
            rp_.push_back(Eigen::Vector3d::Zero());
            //
            dT_from_prev_axis.push_back(Eigen::Matrix4d::Zero());
            dR_from_prev_axis.push_back(Eigen::Matrix3d::Zero());
            dp_from_prev_axis.push_back(Eigen::Vector3d::Zero());
        }
        //
        RdR_from_origin_axis.reserve(joint_num_);
        temp_dpe_.reserve(joint_num_);
        dz_.reserve(joint_num_);
        dpe_.reserve(joint_num_);
        for (int i = 0; i < joint_num_; i++) {
            RdR_from_origin_axis.push_back(Eigen::Matrix3d::Zero());
            temp_dpe_.push_back(Eigen::Vector3d::Zero());
            dz_.push_back(Eigen::Vector3d::Zero());
            dpe_.push_back(Eigen::Vector3d::Zero());
        }
    };
    ~RobotKinematics() {};
    /*
     *  1. initialize
     *
     *  2. Loop
     *  >> 2.1 update_state (q) or (q, dq)
     *
     *  >> 2.2 calc_Jacobian_matrix
     *     Need: update q
     *  >>(2.3 OP) calc_time_derivative_of_Jacobian_matrix
     *     Need: update q and dq
     *  >> 2.4 calc_FK
     */
    void initialize(std::map<int, DHParam> dh_param_map);
    //
    void update_state(Eigen::VectorXd joint_q);
    void update_state(Eigen::VectorXd joint_q, Eigen::VectorXd joint_dq);
    //
    void calc_basic_Jacobian_matrix(Eigen::MatrixXd& jacobian_matrix);
    void calc_time_derivative_of_basic_Jacobian_matrix(
        Eigen::MatrixXd& jacobian_matrix_dot);
    void calc_FK(Eigen::Vector3d& pos, Eigen::Matrix3d& rot);
    void calc_FK(Eigen::VectorXd& q, Eigen::Vector3d& pos,
                 Eigen::Matrix3d& rot);
    void calc_IK(Eigen::Vector3d& pos, Eigen::Matrix3d& rot,
                 Eigen::VectorXd& q0, Eigen::VectorXd& q);
    void calc_pdot_omegadot(Eigen::Vector3d& p_dot, Eigen::Vector3d& omega_dot);

    void get_tool_pos_rot_from_origin_axis(Eigen::Vector3d& pos,
                                           Eigen::Matrix3d& rot) {
        pos = tool_pos;
        rot = tool_rot;
    }
};