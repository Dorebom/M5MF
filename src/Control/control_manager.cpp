#include "control_manager.hpp"

bool ControlManager::initialize(CommCan* can_driver) {
    // Initialize Servo Module
    // >> Set CAN Driver
    servo_.init(can_driver);

    return true;
}
void ControlManager::update() {
    // 1. Get State

    // 2. Execute Cmd

    // 3. Update State

    // 4. Send System
    set_control_state2stack();

    return;
}

void ControlManager::cmd_executor() {
}

std::shared_ptr<NodeStateStack> ControlManager::get_control_state_ptr() {
    return state_stack_;
}

std::shared_ptr<NodeCmdStack> ControlManager::get_control_cmd_ptr() {
    return cmd_stack_;
}

void ControlManager::set_control_state2stack() {
    state_stack_->state_stack_.push(outer_state_);
}
