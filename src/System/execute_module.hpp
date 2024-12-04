#pragma once

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
    // >> Executing Function
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
    void execute(M5MF_CMD_LIST cmd_type, st_node_cmd* cmd);
};