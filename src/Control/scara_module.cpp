#include "scara_module.hpp"

void ScaraModule::forward_kinematics(double* q_state, double* p_state) {
    double s1 = sin(q_state[0]);
    double c1 = cos(q_state[0]);

    double s12 = sin(q_state[0] + q_state[1]);
    double c12 = cos(q_state[0] + q_state[1]);

    double s123 = sin(q_state[0] + q_state[1] + q_state[2]);
    double c123 = cos(q_state[0] + q_state[1] + q_state[2]);

    p_state[0] =
        SCARA_INFO.L1 * s1 + SCARA_INFO.L2 * s12 + SCARA_INFO.L3 * s123;  // x
    p_state[1] =
        SCARA_INFO.L1 * c1 + SCARA_INFO.L2 * c12 + SCARA_INFO.L3 * c123;  // y
    p_state[2] = q_state[0] + q_state[1] + q_state[2];
}

void ScaraModule::inverse_kinematics(double* p_state, double* q0_state,
                                     double* q_state) {
    double xp = p_state[0] - SCARA_INFO.L3 * sin(p_state[2]);
    double yp = p_state[1] + SCARA_INFO.L3 * cos(p_state[2]);

    double l = sqrt(xp * xp + yp * yp);
    double psi = atan2(yp, xp);
    double phi = acos((SCARA_INFO.L1 * SCARA_INFO.L1 + l * l -
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

void ScaraModule::calc_jacobian(double* q_state, double* J) {
    double s1 = sin(q_state[0]);
    double c1 = cos(q_state[0]);

    double s12 = sin(q_state[0] + q_state[1]);
    double c12 = cos(q_state[0] + q_state[1]);

    double s123 = sin(q_state[0] + q_state[1] + q_state[2]);
    double c123 = cos(q_state[0] + q_state[1] + q_state[2]);

    J[0] = SCARA_INFO.L1 * c1 + SCARA_INFO.L2 * c12 + SCARA_INFO.L3 * c123;
    J[1] = SCARA_INFO.L2 * c12 + SCARA_INFO.L3 * c123;
    J[2] = SCARA_INFO.L3 * c123;

    J[3] = SCARA_INFO.L1 * s1 + SCARA_INFO.L2 * s12 + SCARA_INFO.L3 * s123;
    J[4] = SCARA_INFO.L2 * s12 + SCARA_INFO.L3 * s123;
    J[5] = SCARA_INFO.L3 * s123;

    J[6] = 1;
    J[7] = 1;
    J[8] = 1;
}

void ScaraModule::update_state(LocalControlState& state) {
    // FK
    forward_kinematics(state.act_joint_position, state.act_mf_position);
    double J[9];
    calc_jacobian(state.act_joint_position, J);
    state.act_mf_velocity[0] = J[0] * state.act_joint_velocity[0] +
                               J[1] * state.act_joint_velocity[1] +
                               J[2] * state.act_joint_velocity[2];
    state.act_mf_velocity[1] = J[3] * state.act_joint_velocity[0] +
                               J[4] * state.act_joint_velocity[1] +
                               J[5] * state.act_joint_velocity[2];
    state.act_mf_velocity[2] = J[6] * state.act_joint_velocity[0] +
                               J[7] * state.act_joint_velocity[1] +
                               J[8] * state.act_joint_velocity[2];
}
