#include "system_manager.hpp"

SystemManager::SystemManager() {
    // Cmd Stack
    sys_cmd_stack = std::make_shared<NodeCmdStack>(MAX_NODE_CMD_STACK_SIZE);

    // Outer State
    outer_system_state_.state_code.node_id = SYSTEM_NODE_ID;
    outer_system_state_.state_code.data_size = sizeof(SystemState);
    inner_system_state_ = (SystemState*)outer_system_state_.data;
    outer_system_state_.state_code.state_machine =
        node_state_machine::UNCONFIGURED;
    outer_system_state_.state_code.transit_destination_node_state_machine =
        (node_state_machine)SYSTEM_NODE_SIGNATURE;
    // Inner State
    inner_system_state_->init();
    //
    return;
}

SystemManager::~SystemManager() {
    return;
}

bool SystemManager::initialize() {
    return true;
}

bool SystemManager::initialize(CommUdp* udp) {
    udp_ = udp;
    is_init_udp_module = true;

    ext_comm_.initialize(sys_cmd_stack, control_state_stack_,
                         &outer_system_state_, &outer_control_state_, udp_);

    return true;
}

void SystemManager::update() {
    // 1. Set args

    // 2. Execute Cmd
    cmd_executor();

    // 3. Update State

    // 4. Display
    // >> >> >> >> NOTE: Display is not implemented yet << << << <<

    // 5. Send External System
    ext_comm_.send();

    return;
}

void SystemManager::change_sm_initilizing() {
    outer_system_state_.state_code.state_machine =
        node_state_machine::INITIALIZING;
}
void SystemManager::change_sm_ready() {
    outer_system_state_.state_code.state_machine = node_state_machine::READY;
}
void SystemManager::change_sm_stable() {
    outer_system_state_.state_code.state_machine = node_state_machine::STABLE;
}
void SystemManager::change_sm_repair() {
    outer_system_state_.state_code.state_machine = node_state_machine::REPAIR;
}
void SystemManager::change_sm_force_stop() {
    outer_system_state_.state_code.state_machine =
        node_state_machine::FORCE_STOP;
}

bool SystemManager::check_force_stop() {
    if (outer_system_state_.state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return true;
    } else {
        return false;
    }
}

void SystemManager::recv_from_external_system() {
    ext_comm_.recv();
}

void SystemManager::cmd_executor() {
    st_node_cmd cmd;

    if (sys_cmd_stack->cmd_stack_.size() != 0) {
        cmd = sys_cmd_stack->cmd_stack_.pop();

        inner_system_state_->act_cmd_type = cmd.cmd_code.cmd_type;
    }
}