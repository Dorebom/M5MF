#pragma once

#include <memory>

#include "Common/m5mf_cmd_list.hpp"
#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
//
#include "System/st_system_state.hpp"

class ExecuteModule {
private:
    /* data */
    node_state* outer_system_state_;
    SystemState* inner_system_state_;

    std::shared_ptr<NodeCmdStack> cmd_stack_to_external_system;  // 借り物

    // >> Flag
    bool is_obtained_cmd_stack_to_external_system = false;

    /*
     * Executing Function
     */
    // >> CONNECT UDP
    void connect();
    void disconnect();
    // >> >> W.R.T State Machine
    void change_sm_force_stop();
    void change_sm_ready();
    void change_sm_repair();
    void change_sm_stable();
    void release_force_stop();
    // >> ALERT and ERROR
    void reset_alert();
    void reset_error();
    // >> Logging
    void start_logging();
    void stop_logging();
    // >> Data Stream
    void start_stream_state();
    void stop_stream_state();
    void request_state();
    // >> Control System
    void stop_control_system();

    /*
     * Send cmd to External System
     */
    void set_udp_send_cmd(st_node_cmd* cmd);
    // >> Response (No Data Struct)
    void set_response_cmd_start_logging();
    void set_response_cmd_stop_logging();
    void set_response_cmd_connect();
    void set_response_cmd_disconnect();
    // >> Response (Need Data Struct)

public:
    ExecuteModule(/* args */) {
    }
    ~ExecuteModule() {
    }
    void initialize(node_state* outer_system_state,
                    SystemState* inner_system_state) {
        outer_system_state_ = outer_system_state;
        inner_system_state_ = inner_system_state;
    }
    void set_cmd_stack(std::shared_ptr<NodeCmdStack> cmd_stack) {
        cmd_stack_to_external_system = cmd_stack;
        is_obtained_cmd_stack_to_external_system = true;
    }

    void execute(M5MF_CMD_LIST cmd_type, st_node_cmd* cmd);
};