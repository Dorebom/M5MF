#include "execute_module.hpp"

void ExecuteModule::execute(M5MF_CMD_LIST cmd_type, st_node_cmd* cmd) {
    if ((int)cmd_type >= 100) {
        return;
    }

    switch (cmd_type) {
        case M5MF_CMD_LIST::CHANGE_SM_FORCE_STOP:
            change_sm_force_stop();
            break;
        case M5MF_CMD_LIST::CHANGE_SM_READY:
            change_sm_ready();
            break;
        case M5MF_CMD_LIST::CHANGE_SM_REPAIR:
            change_sm_repair();
            break;
        case M5MF_CMD_LIST::CHANGE_SM_STABLE:
            change_sm_stable();
            break;
        case M5MF_CMD_LIST::RELEASE_FORCE_STOP:
            release_force_stop();
            break;
        case M5MF_CMD_LIST::RESET_ALERT:
            reset_alert();
            break;
        case M5MF_CMD_LIST::RESET_ERROR:
            reset_error();
            break;
        case M5MF_CMD_LIST::START_STREAM_STATE:
            start_stream_state();
            break;
        case M5MF_CMD_LIST::STOP_STREAM_STATE:
            stop_stream_state();
            break;
        case M5MF_CMD_LIST::START_LOGGING:
            start_logging();
            break;
        case M5MF_CMD_LIST::STOP_LOGGING:
            stop_logging();
            break;
        case M5MF_CMD_LIST::REQUEST_STATE:
            request_state();
            break;
        default:
            break;
    }
}

void ExecuteModule::change_sm_force_stop() {
    if (outer_system_state_->state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return;
    }
    outer_system_state_->state_code.state_machine =
        node_state_machine::FORCE_STOP;
}

void ExecuteModule::change_sm_ready() {
    if (outer_system_state_->state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return;
    }
    outer_system_state_->state_code.state_machine = node_state_machine::READY;
}

void ExecuteModule::change_sm_repair() {
    if (outer_system_state_->state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return;
    }
    if (inner_system_state_->is_occured_error_) {
        return;
    }
    outer_system_state_->state_code.state_machine = node_state_machine::REPAIR;
}

void ExecuteModule::change_sm_stable() {
    if (outer_system_state_->state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return;
    }
    if (inner_system_state_->is_occured_error_ ||
        inner_system_state_->is_occured_warning_) {
        return;
    }
    outer_system_state_->state_code.state_machine = node_state_machine::STABLE;
}

void ExecuteModule::release_force_stop() {
    // マニュアル非常停止が入ってる場合、マニュアル以外での解放はできない
    if (inner_system_state_->is_manual_force_stop ||
        outer_system_state_->state_code.state_machine !=
            node_state_machine::FORCE_STOP) {
        return;
    }
    outer_system_state_->state_code.state_machine = node_state_machine::READY;
}

void ExecuteModule::reset_alert() {
    inner_system_state_->is_occured_warning_ = false;
}

void ExecuteModule::reset_error() {
    inner_system_state_->is_occured_error_ = false;
}

void ExecuteModule::start_logging() {
    if (inner_system_state_->is_logging ||
        !inner_system_state_->is_connected_udp) {
        return;
    }
    inner_system_state_->is_logging = true;
}

void ExecuteModule::stop_logging() {
    if (!inner_system_state_->is_logging ||
        !inner_system_state_->is_connected_udp) {
        return;
    }
    inner_system_state_->is_logging = false;
}

void ExecuteModule::start_stream_state() {
    if (inner_system_state_->is_streaming_state ||
        !inner_system_state_->is_connected_udp) {
        return;
    }
    inner_system_state_->is_streaming_state = true;
}

void ExecuteModule::stop_stream_state() {
    if (!inner_system_state_->is_streaming_state ||
        !inner_system_state_->is_connected_udp) {
        return;
    }
    inner_system_state_->is_streaming_state = false;
}

void ExecuteModule::request_state() {
    if (inner_system_state_->is_requested_state_at_once ||
        !inner_system_state_->is_connected_udp ||
        inner_system_state_->is_streaming_state) {
        return;
    }
    inner_system_state_->is_requested_state_at_once = true;
}
