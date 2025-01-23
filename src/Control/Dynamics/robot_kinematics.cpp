#include "robot_kinematics.hpp"

void RobotKinematics::calc_p_and_R_from_prev_axis() {
    for (int i = 0; i < tool_point_num_; i++) {
        T_from_prev_axis[i] = htmat[i].get_htmat(q_[i]);
        pose_transform_.transform_htmat2pos_rotmat(
            T_from_prev_axis[i], p_from_prev_axis[i], R_from_prev_axis[i]);
    }
}

void RobotKinematics::calc_p_and_R_from_origin_axis() {
    Eigen::Matrix4d T = robot_base_htmat_;
    for (int i = 0; i < tool_point_num_; i++) {
        T *= T_from_prev_axis[i];
        T_from_origin_axis[i] = T;
        pose_transform_.transform_htmat2pos_rotmat(T_from_origin_axis[i],
                                                   p_from_origin_axis[i],
                                                   R_from_origin_axis[i]);
    }
}

Eigen::RowVectorXd RobotKinematics::calc_error_pose(
    Eigen::Vector3d src_pos, Eigen::Matrix3d src_rotmat,
    Eigen::Vector3d dst_pos, Eigen::Matrix3d dst_rotmat) {
    Eigen::Vector3d err_pos = src_pos - dst_pos;
    Eigen::Vector3d err_rotvec = Eigen::Vector3d::Zero();

    Eigen::Matrix3d err_rotmat = src_rotmat.transpose() * dst_rotmat;
    //
    if (err_rotmat(0, 0) * err_rotmat(1, 1) * err_rotmat(2, 2) > 0.9999997) {
        if (err_rotmat(0, 0) + err_rotmat(1, 1) + err_rotmat(2, 2) >
            2.9999997) {
            Eigen::Vector3d err_rotvec = Eigen::Vector3d::Zero();
        } else {
            err_rotvec[0] = EIGEN_PI * 0.5 * (err_rotmat(0, 0) - 1.0);
            err_rotvec[1] = EIGEN_PI * 0.5 * (err_rotmat(1, 1) - 1.0);
            err_rotvec[2] = EIGEN_PI * 0.5 * (err_rotmat(2, 2) - 1.0);
        }
    } else {
        Eigen::Vector3d err_l;
        err_l << err_rotmat(2, 1) - err_rotmat(1, 2),
            err_rotmat(0, 2) - err_rotmat(2, 0),
            err_rotmat(1, 0) - err_rotmat(0, 1);
        // normalize
        double err_l_norm = err_l.norm();
        err_rotvec = std::atan2(err_l_norm, err_rotmat.trace() - 1.0) * err_l /
                     err_l_norm;
    }
    Eigen::RowVectorXd err_pose(6);
    err_pose << err_pos, err_rotvec;
    return err_pose;
}

void RobotKinematics::calc_z_from_origin_axis() {
    Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
    for (int i = 0; i < tool_point_num_; i++) {
        z_from_origin_axis[i] = R_from_origin_axis[i] * z;
    }
}

void RobotKinematics::calc_pe_from_origin_axis() {
    for (int i = 0; i < joint_num_; i++) {
        rp_[i] = R_from_origin_axis[i] * p_from_prev_axis[i + 1];
    }
    Eigen::Vector3d pe = Eigen::Vector3d::Zero();
    for (int i = 1; i < tool_point_num_; i++) {
        pe += rp_[joint_num_ - i];
        pe_from_origin_axis[joint_num_ - i] = pe;
    }
}

void RobotKinematics::calc_dp_and_dR_from_prev_axis() {
    for (int i = 0; i < tool_point_num_; i++) {
        dT_from_prev_axis[i] = htmat[i].get_dhtmat(q_[i]);
        pose_transform_.transform_htmat2pos_rotmat(
            dT_from_prev_axis[i], dp_from_prev_axis[i], dR_from_prev_axis[i]);
    }
    /*
    // print dp_from_prev_axis
    std::cout << "dT_from_prev_axis" << std::endl;
    for (int i = 0; i < tool_point_num_; i++) {
        std::cout << "dT_from_prev_axis[" << i << "] = " << std::endl;
        std::cout << dT_from_prev_axis[i] << std::endl;
        }
    */
}

void RobotKinematics::calc_RdR_from_origin_axis() {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    for (int i = 0; i < joint_num_; i++) {
        RdR_from_origin_axis[i] = R * dR_from_prev_axis[i];
        R = R_from_origin_axis[i];
    }
}
//

void RobotKinematics::initialize(std::map<int, DHParam> dh_param_map,
                                 Eigen::Vector3d tool_base_pos,
                                 Eigen::Matrix3d tool_base_rot) {
    for (int i = 0; i < joint_num_; i++) {
        htmat[i].set_dhparam(dh_param_map[i].a, dh_param_map[i].alpha,
                             dh_param_map[i].d, dh_param_map[i].theta_offset);
    }
    tool_base_pos_ = tool_base_pos;
    tool_base_rot_ = tool_base_rot;
    htmat[joint_num_].set_dhparam(
        tool_base_pos_, tool_base_rot_,
        0.0);  // 初期化時はツール取り付けフランジ部がツールポイント
}

void RobotKinematics::set_tool_point(Eigen::Vector3d pos, Eigen::Matrix3d rot) {
    tool_pos = pos;
    tool_rot = rot;

    Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
    temp.block<3, 3>(0, 0) = rot;
    temp.block<3, 1>(0, 3) = pos;

    temp = htmat[joint_num_].get_htmat(0.0) * temp;

    Eigen::Vector3d temp_pos = temp.block<3, 1>(0, 3);
    Eigen::Matrix3d temp_rot = temp.block<3, 3>(0, 0);

    htmat[joint_num_].set_dhparam(temp_pos, temp_rot, 0.0);
}

void RobotKinematics::set_robot_base(Eigen::Vector3d pos, Eigen::Matrix3d rot) {
    robot_base_pos_ = pos;
    robot_base_rot_ = rot;
    robot_base_htmat_.block<3, 3>(0, 0) = rot;
    robot_base_htmat_.block<3, 1>(0, 3) = pos;
}

void RobotKinematics::update_state(Eigen::VectorXd joint_q) {
    for (int i = 0; i < joint_num_; i++) {
        q_[i] = joint_q[i];
    }
    calc_p_and_R_from_prev_axis();
    calc_p_and_R_from_origin_axis();
    // print q_
    // for (int i = 0; i < tool_point_num_; i++) {
    //    std::cout << "q_[" << i << "] = " << q_[i] << std::endl;
    //}
}

void RobotKinematics::update_state(Eigen::VectorXd joint_q,
                                   Eigen::Vector3d robot_base_pos,
                                   Eigen::Matrix3d robot_base_rot) {
    set_robot_base(robot_base_pos, robot_base_rot);
    update_state(joint_q);
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
        // temp_m += dq_[i] * dR_from_prev_axis[i];
        temp_m += dq_[i] * RdR_from_origin_axis[i];
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

void RobotKinematics::calc_FK_from_world(Eigen::Vector3d& pos,
                                         Eigen::Matrix3d& rot) {
    pos = p_from_origin_axis[tool_point_num_ - 1];
    rot = R_from_origin_axis[tool_point_num_ - 1];
}

void RobotKinematics::calc_FK_from_world(Eigen::VectorXd& q,
                                         Eigen::Vector3d& pos,
                                         Eigen::Matrix3d& rot) {
    update_state(q);
    calc_FK_from_world(pos, rot);
}

void RobotKinematics::calc_pdot_omegadot(Eigen::Vector3d& p_dot,
                                         Eigen::Vector3d& omega,
                                         Eigen::MatrixXd& jacobian_matrix) {
    Eigen::VectorXd temp = jacobian_matrix * dq_;
    p_dot = temp.block<3, 1>(0, 0);
    omega = temp.block<3, 1>(3, 0);
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Matrix3d>>
RobotKinematics::calc_p_and_R_from_prev_axis(Eigen::VectorXd joint_q) {
    Eigen::VectorXd q(tool_point_num_);
    for (int i = 0; i < joint_num_; i++) {
        q[i] = joint_q[i];
    }

    for (int i = 0; i < tool_point_num_; i++) {
        T_from_prev_axis[i] = htmat[i].get_htmat(q[i]);
        pose_transform_.transform_htmat2pos_rotmat(
            T_from_prev_axis[i], p_from_prev_axis[i], R_from_prev_axis[i]);
    }

    return {p_from_prev_axis, R_from_prev_axis};
}
