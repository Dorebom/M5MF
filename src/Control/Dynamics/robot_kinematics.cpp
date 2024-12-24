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
    for (int i = 1; i < tool_point_num_; i++) {
        T *= T_from_prev_axis[i];
        T_from_origin_axis[i] = T;
        pose_transform_.transform_htmat2pos_rotmat(T_from_origin_axis[i],
                                                   p_from_origin_axis[i],
                                                   R_from_origin_axis[i]);
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

void RobotKinematics::calc_dR_from_origin_axis() {
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    for (int i = 0; i < joint_num_; i++) {
        for (int j = 0; j < i; j++) {
            //
            if (i == j) {
                if (i == 0) {
                    dR_from_origin_axis[j] = dR_from_prev_axis[j];
                } else {
                    dR_from_origin_axis[i * joint_num_ + j] =
                        R_from_origin_axis[j - 1] * dR_from_prev_axis[j];
                }
            } else {
                R = Eigen::Matrix3d::Identity();
                for (int k = 0; k < i - j; k++) {
                    R *= R_from_prev_axis[k + j + 1];
                }
                if (j == 0) {
                    dR_from_origin_axis[i * joint_num_] =
                        dR_from_prev_axis[0] * R;
                } else {
                    dR_from_origin_axis[i * joint_num_ + j] =
                        R_from_origin_axis[j - 1] * dR_from_prev_axis[j] * R;
                }
            }
        }
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
    calc_dR_from_origin_axis();
    // 350Lから
    Eigen::Matrix3d temp_ = Eigen::Matrix3d::Identity();
    for (int i = 0; i < joint_num_; i++) {
        // //temp_dpe
        temp_dpe_[i] = p_from_prev_axis[i + 1];
        for (int j = i + 1; j < joint_num_; j++) {
            temp_ *= R_from_prev_axis[j];
            temp_dpe_[i] = temp_ * p_from_prev_axis[j + 1];
            //
            dz_[j - 1] = dR_from_prev_axis[i] * temp_ *
                         Eigen::Vector3d::UnitZ() * dq_[i];  // ここがおかしい
        }
        //

        /*
        jacobian_matrix_dot.block<3, 1>(0, i) =
            z_from_origin_axis[i].cross(dp_from_prev_axis[i]);
        jacobian_matrix_dot.block<3, 1>(3, i) = z_from_origin_axis[i];
        for (int j = 0; j < joint_num_; j++) {
            jacobian_matrix_dot.block<3, 1>(0, i) +=
                dR_from_origin_axis[i * joint_num_ + j] *
                pe_from_origin_axis[j];
        }
        */
    }
}