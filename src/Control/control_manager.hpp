#pragma once

#include <memory>
#include <string>

#include "Common/m5mf_def.hpp"
#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
//
#include "Control/st_control_state.hpp"
// >> module
#include "Control/servo_module.hpp"
#include "Device/Comm/can_driver.hpp"

class ControlManager {
private:
    // >> Module
    ServoModule servo_;
    // >> State
    node_state outer_state_;
    ControlState *state_;
    std::shared_ptr<NodeCmdStack> cmd_stack_;
    std::shared_ptr<NodeStateStack> state_stack_;

    // >> Function
    void get_state();

public:
    ControlManager() {
        // >> Outer State
        outer_state_.state_code.node_id = CONTROL_NODE_ID;
        outer_state_.state_code.data_size = sizeof(ControlState);
        state_ = (ControlState *)outer_state_.data;
        outer_state_.state_code.state_machine =
            node_state_machine::UNCONFIGURED;
        outer_state_.state_code.transit_destination_node_state_machine =
            (node_state_machine)CONTROL_NODE_SIGNATURE;
        // >> Inner State
        state_->init();
        // >> State Stack
        state_stack_ = std::make_shared<NodeStateStack>(
            MAX_NODE_STATE_STACK_SIZE);  // 外部通信用
        state_stack_->state_stack_.change_overwrite_mode(true);
        // >> Cmd Stack
        cmd_stack_ = std::make_shared<NodeCmdStack>(MAX_NODE_CMD_STACK_SIZE);
        cmd_stack_->cmd_stack_.set_name("ControlCmdStack");
        // >> Initialize Data
        state_->state_code.mf_type = MECHANICAL_FRAME_LIST::ALLJOINT;
        state_->state_code.ctrl_mode = CTRL_MODE_LIST::STAY;
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