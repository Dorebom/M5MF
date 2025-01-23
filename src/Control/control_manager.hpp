#pragma once

#include <memory>
#include <string>

#include "Common/m5mf_def.hpp"
#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
//
#include "Control/st_control_cmd.hpp"
#include "Control/st_control_state.hpp"
// >> module
#include "Control/Dynamics/rigid_transform.hpp"
#include "Control/Dynamics/robot_dynamics.hpp"
#include "Control/Dynamics/robot_kinematics.hpp"
#include "Control/scara_def.hpp"
#include "Control/scara_module.hpp"
#include "Control/servo_def.hpp"
#include "Control/servo_module.hpp"
#include "Device/Comm/can_driver.hpp"
// #define ENABLE_EXT_FORCE

class ControlManager {
private:
    // >> Module
    ServoModule servo_;
    ScaraModule scara_;

    // 軸数3の場合
    RobotKinematics robot_kinematics_;
    RobotDynamics robot_dynamics_;

    // >> State
    LocalControlState local_state, prev_local_state;
    ControlState *state_;
    std::shared_ptr<NodeCmdStack> cmd_stack_;
    std::shared_ptr<NodeStateStack> state_stack_;
    //
    std::vector<Eigen::Vector3d> p_from_prev_axis;
    std::vector<Eigen::Matrix3d> R_from_prev_axis;
    Eigen::MatrixXd M;
    Eigen::VectorXd h;
    Eigen::VectorXd g;

    // >> Data
    uint8_t cmd_watchdog_counter = 0;
    const ServoInfo SERVO_INFO;
    const ScaraInfo SCARA_INFO;
    // >> Function
    void get_state();
    void change_data_size();
    void check_safety();

public:
    ControlManager()
        : robot_dynamics_(SERVO_NUM), robot_kinematics_(SERVO_NUM) {
        // >> State Stack
        state_stack_ = std::make_shared<NodeStateStack>(
            MAX_NODE_STATE_STACK_SIZE);  // 外部通信用
        state_stack_->state_stack_.change_overwrite_mode(true);
        state_stack_->state_stack_.set_name("ControlStateStack");
        // >> Cmd Stack
        cmd_stack_ = std::make_shared<NodeCmdStack>(MAX_NODE_CMD_STACK_SIZE);
        cmd_stack_->cmd_stack_.set_name("ControlCmdStack");
        // >> Initialize Data
        local_state.state_code.mf_type = MECHANICAL_FRAME_LIST::ALLJOINT;
        local_state.state_code.ctrl_mode = CTRL_MODE_LIST::STAY;
        local_state.state_code.timestamp = 0;
        local_state.state_code.is_force_stop = false;
        local_state.state_code.is_connecting_device = false;
        local_state.state_code.is_power_on = false;
        local_state.state_code.data_size = sizeof(MFAllJointPosState);
        // >> Robot Kinematics
        std::map<int, DHParam> dh_param_;
        DHParam dh_param;
        dh_param.a = 0.0;
        dh_param.alpha = 0.0;
        dh_param.d = 0.2;
        dh_param.theta_offset = 0.0;
        dh_param_[0] = dh_param;
        dh_param.a = SCARA_INFO.L1;
        dh_param.alpha = 0.0;
        dh_param.d = 0.0;
        dh_param.theta_offset = 0.0;
        dh_param_[1] = dh_param;
        dh_param.a = SCARA_INFO.L2;
        dh_param.alpha = 0.0;
        dh_param.d = 0.0;
        dh_param.theta_offset = 0.0;
        dh_param_[2] = dh_param;
        // tool base pos
        Eigen::Vector3d tool_base_pos = Eigen::Vector3d::Zero();
        tool_base_pos(0) = SCARA_INFO.L3;
        robot_kinematics_.initialize(dh_param_, tool_base_pos,
                                     Eigen::Matrix3d::Identity());
        robot_kinematics_.set_robot_base(Eigen::Vector3d::Zero(),
                                         Eigen::Matrix3d::Identity());
        Eigen::Vector3d tool_point_pos =
            Eigen::Vector3d::Zero();  // tool_base_pos == tool_point_pos
        tool_point_pos(0) = SCARA_INFO.L3;
        robot_kinematics_.set_tool_point(tool_point_pos,
                                         Eigen::Matrix3d::Identity());
        p_from_prev_axis.reserve(4);
        R_from_prev_axis.reserve(4);
        Eigen::VectorXd q = Eigen::VectorXd::Zero(3);
        robot_kinematics_.calc_p_and_R_from_prev_axis(q);
        std::tie(p_from_prev_axis, R_from_prev_axis) =
            robot_kinematics_.get_p_and_R_from_prev_axis();
        // >> Robot Dynamics
        robot_dynamics_.set_param(p_from_prev_axis, SCARA_INFO.s_i_i,
                                  SCARA_INFO.I_i_i, SCARA_INFO.m);
        M = Eigen::MatrixXd::Zero(3, 3);
        h = Eigen::VectorXd::Zero(3);
        g = Eigen::VectorXd::Zero(3);
    };
    ~ControlManager() {};
    bool initialize(CommCan *can_driver);
    void update();
    void cmd_executor();
    //
    std::shared_ptr<NodeStateStack> get_control_state_ptr();
    std::shared_ptr<NodeCmdStack> get_control_cmd_ptr();
    void set_control_state2stack();
};