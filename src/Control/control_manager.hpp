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
#include "Control/servo_def.hpp"
#include "Control/servo_module.hpp"
#include "Device/Comm/can_driver.hpp"

class ControlManager {
private:
    // >> Module
    ServoModule servo_;
    // >> State
    LocalControlState local_state, prev_local_state;
    ControlState *state_;
    std::shared_ptr<NodeCmdStack> cmd_stack_;
    std::shared_ptr<NodeStateStack> state_stack_;

    // >> Data
    uint8_t cmd_watchdog_counter = 0;
    const ServoInfo SERVO_INFO;
    // >> Function
    void get_state();
    void change_data_size();
    void check_safety();

public:
    ControlManager() {
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