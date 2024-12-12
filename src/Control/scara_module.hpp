#pragma once

#include "st_control_state.hpp"
//
#include "scara_def.hpp"

class ScaraModule {
private:
    /* data */
    ScaraInfo SCARA_INFO;

    // >> Function
    void forward_kinematics(double* q_state, double* p_state);
    void inverse_kinematics(double* p_state, double* q0_state, double* q_state);

public:
    ScaraModule(/* args */) {
    }
    ~ScaraModule() {
    }
    void update_state(LocalControlState& state);
};
