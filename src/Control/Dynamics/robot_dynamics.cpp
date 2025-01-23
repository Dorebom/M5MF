#include "robot_dynamics.hpp"

void RobotDynamics::_calc_inv_dynamics_step_1() {
    // calc omega_i_i
    omega_i_i[0] = ext_omega_0_0_;
    for (size_t i = 0; i < joint_num_; i++) {
        omega_i_i[i + 1] = R_from_prev_axis[i].transpose() * omega_i_i[i] +
                           dq_[i] * Eigen::Vector3d::UnitZ();
    }
    // calc omega_dot_i_i
    omega_dot_i_i[0] = ext_dot_omega_0_0_;
    for (size_t i = 0; i < joint_num_; i++) {
        omega_dot_i_i[i + 1] =
            R_from_prev_axis[i].transpose() * omega_dot_i_i[i] +
            ddq_[i] * Eigen::Vector3d::UnitZ() +
            (R_from_prev_axis[i].transpose() * omega_i_i[i])
                .cross(dq_[i] * Eigen::Vector3d::UnitZ());
    }
    // calc p_ddot_i_i
    p_ddot_i_i[0] = ext_ddot_p_0_0_;
    for (size_t i = 0; i < joint_num_; i++) {
        p_ddot_i_i[i + 1] =
            R_from_prev_axis[i].transpose() *
            (p_ddot_i_i[i] + omega_dot_i_i[i].cross(p_from_prev_axis[i]) +
             omega_i_i[i].cross(omega_i_i[i].cross(p_from_prev_axis[i])));
    }
}

void RobotDynamics::_calc_inv_dynamics_step_2() {
    for (size_t i = 0; i < joint_num_; i++) {
        // calc s_ddot_i_i
        s_ddot_i_i[i] =
            p_ddot_i_i[i] + omega_dot_i_i[i].cross(s_hat_i_i_[i]) +
            omega_i_i[i].cross(omega_i_i[i].cross(p_from_prev_axis[i]));
        // calc f_hat_i_i
        f_hat_i_i[i] = m_[i] * s_ddot_i_i[i];
        // calc n_hat_i_i
        n_hat_i_i[i] = I_i_i_[i] * omega_dot_i_i[i] +
                       omega_i_i[i].cross(I_i_i_[i] * omega_i_i[i]);
    }
}

void RobotDynamics::_calc_inv_dynamics_step_3() {
    // joint_num_ + 1 = tool_point_num_
    //
    // calc f_i_i
    f_i_i[joint_num_] = ext_f_E_E_;
    for (int i = joint_num_ - 1; i >= 0; i--) {
        f_i_i[i] = R_from_prev_axis[i] * f_i_i[i + 1] + f_hat_i_i[i];
    }
    // calc n_i_i
    n_i_i[joint_num_] = ext_n_E_E_;
    for (int i = joint_num_ - 1; i >= 0; i--) {
        n_i_i[i] =
            R_from_prev_axis[i] * n_i_i[i + 1] + n_hat_i_i[i] +
            p_from_prev_axis[i].cross(R_from_prev_axis[i] * f_i_i[i + 1]) +
            s_hat_i_i_[i].cross(f_hat_i_i[i]);
    }
    // display f and n
}

void RobotDynamics::calc_tau(
    const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
    const std::vector<Eigen::Matrix3d>& R, const Eigen::Vector3d& ext_omega_0_0,
    const Eigen::Vector3d& ext_dot_omega_0_0,
    const Eigen::Vector3d& ext_ddot_p_0_0, const Eigen::Vector3d& ext_f_E_E,
    const Eigen::Vector3d& ext_n_E_E, Eigen::VectorXd& tau) {
    // update state
    update_state(dq, ddq, R, ext_omega_0_0, ext_dot_omega_0_0, ext_ddot_p_0_0,
                 ext_f_E_E, ext_n_E_E);
    // step 1
    _calc_inv_dynamics_step_1();
    // step 2
    _calc_inv_dynamics_step_2();
    // step 3
    _calc_inv_dynamics_step_3();
    // set tau
    for (size_t i = 0; i < joint_num_; i++) {
        tau[i] = Eigen::Vector3d::UnitZ().dot(n_i_i[i]);
    }
    /*
    for (size_t i = 0; i < joint_num_; i++)
    {
            // print n_i_i
            std::cout << "n_i_i[" << i << "] = " << n_i_i[i] << std::endl;
            // print f_i_i
            std::cout << "f_i_i[" << i << "] = " << f_i_i[i] << std::endl;
            // print tau
            std::cout << "tau[" << i << "] = " << tau[i] << std::endl;
    }
    */
}
