#include "scara_module.hpp"

void ScaraModule::forward_kinematics(double* q_state, double* p_state) {
    double s1 = std::sin(q_state[0]);
    double c1 = std::cos(q_state[0]);

    double s12 = std::sin(q_state[0] + q_state[1]);
    double c12 = std::cos(q_state[0] + q_state[1]);

    double s123 = std::sin(q_state[0] + q_state[1] + q_state[2]);
    double c123 = std::cos(q_state[0] + q_state[1] + q_state[2]);

    p_state[0] =
        SCARA_INFO.L1 * s1 + SCARA_INFO.L2 * s12 + SCARA_INFO.L3 * s123;  // x
    p_state[1] =
        SCARA_INFO.L1 * c1 + SCARA_INFO.L2 * c12 + SCARA_INFO.L3 * c123;  // y
    p_state[2] = q_state[0] + q_state[1] + q_state[2];
}

void ScaraModule::inverse_kinematics(double* p_state, double* q0_state,
                                     double* q_state) {
    double xp = p_state[0] - SCARA_INFO.L3 * std::sin(p_state[2]);
    double yp = p_state[1] + SCARA_INFO.L3 * std::cos(p_state[2]);

    double l = std::sqrt(xp * xp + yp * yp);
    double psi = std::atan2(yp, xp);
    double phi = std::acos((SCARA_INFO.L1 * SCARA_INFO.L1 + l * l -
                            SCARA_INFO.L2 * SCARA_INFO.L2) /
                           (2 * SCARA_INFO.L1 * l));
    if (q0_state[1] > 0) {
        q_state[0] = psi + phi;
        q_state[1] = q0_state[1] - phi;
    } else {
        q_state[0] = psi - phi;
        q_state[1] = q0_state[1] + phi;
    }
}

void ScaraModule::update_state(LocalControlState& state) {
    // FK
    forward_kinematics(state.act_joint_position, state.act_mf_position);
}