#pragma once

#include <memory>

// Common Shared System
#include "Common/node_cmd.hpp"
#include "Common/node_def.hpp"
#include "Common/node_state.hpp"
// System
#include "st_system_state.hpp"
// Control
#include "Control/st_control_state.hpp"
// module
#include "Device/Comm/udp_driver.hpp"
// >> inner module
#include "display_module.hpp"
#include "execute_module.hpp"
#include "ext_comm_module.hpp"
#include "forcestop_module.hpp"
#include "heartbeat_module.hpp"

class SystemManager {
private:
    // >> Module
    CommUdp* udp_;
    ExtCommModule ext_comm_;
    HeartbeatModule heartbeat_;
    DisplayModule display_;
    ForceStopModule force_stop_;
    ExecuteModule execute_module_;

    // >> State
    node_state outer_system_state_;
    node_state outer_control_state_;
    SystemState* inner_system_state_;
    ControlState* inner_control_state_;

    // >> Stack
    std::shared_ptr<NodeCmdStack> sys_cmd_stack;
    std::shared_ptr<NodeCmdStack> ctrl_cmd_stack;
    // // 外部通信用
    std::shared_ptr<NodeStateStack> control_state_stack_;

    // >> Flag
    bool is_init_main_task = false;
    bool is_init_ctrl_task = false;
    bool is_init_udp_module = false;
    bool is_heartbeat_high = false;

    // >> Function
    // >> >> W.R.T Execute Command
    void cmd_executor();
    void transfer_cmd_to_control(st_node_cmd* cmd);

public:
    SystemManager();
    ~SystemManager();
    bool initialize(int time_interval);
    bool initialize(int time_interval, CommUdp* udp);
    void update();

    // >> Operate State Machine
    //    定義はCommon/node_state_machine.hppに記載
    void change_sm_initilizing();
    void change_sm_ready();
    void change_sm_stable();
    void change_sm_repair();
    void change_sm_force_stop();

    // >> Get Inner System State
    bool check_force_stop();
    bool check_initilized_udp_module() {
        return is_init_udp_module;
    }
    bool check_initilized_main_task() {
        return is_init_main_task;
    }
    bool check_initilized_ctrl_task() {
        return is_init_ctrl_task;
    }
    // >> w.r.t Control system
    void set_initialized_ctrl_task() {
        is_init_ctrl_task = true;
    }
    void set_control_cmd_ptr(std::shared_ptr<NodeCmdStack> cmd_stack) {
        ctrl_cmd_stack = cmd_stack;
    }
    void set_control_state_ptr(std::shared_ptr<NodeStateStack> state_stack) {
        control_state_stack_ = state_stack;
    }
    // >> w.r.t Manual Force Stop
    void set_manual_force_stop_signal(bool is_force_stop) {
        if (is_force_stop) {
            force_stop_.set_manual_force_stop();
        } else {
            force_stop_.release_manual_force_stop();
        }
        inner_system_state_->is_manual_force_stop = is_force_stop;
    }
    // >> w.r.t Display
    void set_canvas(M5Canvas* canvas) {
        display_.set_canvas(canvas);
    }

    // >> Get Outer System Status and Command
    void recv_from_external_system();
};