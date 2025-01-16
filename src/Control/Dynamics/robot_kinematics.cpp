#include "robot_kinematics.hpp"

void RobotKinematics::calc_p_and_R_from_prev_axis() {
    for (int i = 1; i < tool_point_num_; i++) {
        T_from_prev_axis[i] = htmat[i].get_htmat(q_[i]);
        pose_transform_.transform_htmat2pos_rotmat(
            T_from_prev_axis[i], p_from_prev_axis[i], R_from_prev_axis[i]);
    }
}

void RobotKinematics::calc_p_and_R_from_origin_axis() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T_from_origin_axis[0] = T;
    pose_transform_.transform_htmat2pos_rotmat(
        T_from_origin_axis[0], p_from_origin_axis[0], R_from_origin_axis[0]);
    for (int i = 1; i < tool_point_num_; i++) {
        T *= T_from_prev_axis[i];
        T_from_origin_axis[i] = T;
        pose_transform_.transform_htmat2pos_rotmat(T_from_origin_axis[i],
                                                   p_from_origin_axis[i],
                                                   R_from_origin_axis[i]);
        // Joint J1 -> i = 1
    }
}

void RobotKinematics::calc_z_from_origin_axis() {
    Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
    for (int i = 1; i < tool_point_num_; i++) {
        z_from_origin_axis[i] = R_from_origin_axis[i] * z;
    }
}

void RobotKinematics::calc_pe_from_origin_axis() {
    for (int i = 1; i < joint_num_; i++) {
        rp_[i] = R_from_origin_axis[i] * p_from_prev_axis[i + 1];
    }
    Eigen::Vector3d pe = Eigen::Vector3d::Zero();
    for (int i = 1; i < tool_point_num_; i++) {
        pe += rp_[joint_num_ - i];
        pe_from_origin_axis[joint_num_ - i] = pe;
    }
}

void RobotKinematics::calc_dp_and_dR_from_prev_axis() {
    for (int i = 1; i < tool_point_num_; i++) {
        dT_from_prev_axis[i] = htmat[i].get_dhtmat(q_[i]);
        pose_transform_.transform_htmat2pos_rotmat(
            dT_from_prev_axis[i], dp_from_prev_axis[i], dR_from_prev_axis[i]);
    }
}

void RobotKinematics::calc_RdR_from_origin_axis() {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    for (int i = 1; i < joint_num_; i++) {
        RdR_from_origin_axis[i] = R * dR_from_prev_axis[i];
        R = R_from_origin_axis[i];
    }
}
//

void RobotKinematics::initialize(std::map<int, DHParam> dh_param_map) {
    for (int i = 0; i < tool_point_num_; i++) {
        htmat[i].set_dhparam(dh_param_map[i].a, dh_param_map[i].alpha,
                             dh_param_map[i].d, dh_param_map[i].theta_offset);
    }
}

void RobotKinematics::update_state(Eigen::VectorXd joint_q) {
    for (int i = 0; i < joint_num_; i++) {
        q_[i] = joint_q[i];
    }
    calc_p_and_R_from_prev_axis();
    calc_p_and_R_from_origin_axis();
}

void RobotKinematics::update_state(Eigen::VectorXd joint_q,
                                   Eigen::VectorXd joint_dq) {
    for (int i = 0; i < joint_num_; i++) {
        q_[i] = joint_q[i];
        dq_[i] = joint_dq[i];
    }
    calc_p_and_R_from_prev_axis();
    calc_p_and_R_from_origin_axis();
};

void RobotKinematics::calc_basic_Jacobian_matrix(
    Eigen::MatrixXd& jacobian_matrix) {
    calc_z_from_origin_axis();
    calc_pe_from_origin_axis();
    for (int i = 0; i < joint_num_; i++) {
        jacobian_matrix.block<3, 1>(0, i) =
            z_from_origin_axis[i].cross(pe_from_origin_axis[i]);
        jacobian_matrix.block<3, 1>(3, i) = z_from_origin_axis[i];
    }
}

void RobotKinematics::calc_time_derivative_of_basic_Jacobian_matrix(
    Eigen::MatrixXd& jacobian_matrix_dot) {
    calc_dp_and_dR_from_prev_axis();
    calc_RdR_from_origin_axis();  // calc {0}^R_{i-1}{i-1}^dR_{i}
    //
    for (int i = 0; i < joint_num_; i++) {
        dz_[i] = Eigen::Vector3d::Zero();
        dpe_[i] = Eigen::Vector3d::Zero();
    }
    Eigen::Matrix3d temp_m = Eigen::Matrix3d::Zero();
    Eigen::Vector3d temp_v = Eigen::Vector3d::Zero();
    // calc dz and dpe
    for (int i = 0; i < joint_num_; i++) {
        temp_m *= R_from_prev_axis[i];
        temp_m += dq_[i] * dR_from_prev_axis[i];
        dz_[i] = temp_m * Eigen::Vector3d::UnitZ();
        temp_v = temp_m * p_from_prev_axis[i + 1];
        for (int j = 0; j <= i; j++) {
            dpe_[j] += temp_v;
        }
    }
    // calc jacobian_matrix_dot
    for (int i = 0; i < joint_num_; i++) {
        jacobian_matrix_dot.block<3, 1>(0, i) =
            dz_[i].cross(pe_from_origin_axis[i]) +
            z_from_origin_axis[i].cross(dpe_[i]);
        jacobian_matrix_dot.block<3, 1>(3, i) = dz_[i];
    }
}

void RobotKinematics::calc_pdot_omegadot(Eigen::Vector3d& p_dot,
                                         Eigen::Vector3d& omega_dot) {
}
