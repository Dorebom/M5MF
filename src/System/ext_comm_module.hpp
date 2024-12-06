#pragma once

#include <memory>

#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
#include "Common/st_udp_data_frame.hpp"
#include "System/st_system_state.hpp"
//
#include "Control/st_control_state.hpp"
#include "Control/st_control_state_limited.hpp"
//
#include "Device/Comm/udp_driver.hpp"

class ExtCommModule {
private:
    // module
    CommUdp* udp_;

    /* data */
    std::shared_ptr<NodeCmdStack> sys_cmd_stack_;  // 借り物
    // // 外部通信用
    std::shared_ptr<NodeStateStack> control_state_stack_;         // 借り物
    std::shared_ptr<NodeCmdStack> cmd_stack_to_external_system_;  // 持ち物

    node_state* outer_system_state_;
    node_state* outer_control_state_;
    SystemState* inner_system_state_;
    //
    node_state outer_control_state_limited_;
    ControlStateLimited* control_state_limited_;

    // >> UDP Packet
    uint8_t send_packet_buffer[UDP_SEND_PACKET_MAX_SIZE];
    uint8_t recv_packet_buffer[UDP_RECV_PACKET_MAX_SIZE];
    // for udp
    UdpDataFrame ext_send_packet_;  // TODO node class への移植
    uint8_t unsent_data[MAX_STACK_SIZE_AT_ONCE];  // TODO node class への移植
    int unsent_stack_marker = 0;  // TODO node class への移植

    // >> Flag
    bool is_unsent_data = false;  // For UDP
    bool is_prev_logging = false;
    bool is_prev_connected_udp = false;

    // >> Function
    int generate_send_packet();
    void set_udp_send_state(st_node_state* state);
    void set_udp_send_cmd(st_node_cmd* cmd);
    void reset_udp_send_packet(bool discard_unsent_data);
    void set_response_cmd_start_logging();
    void set_response_cmd_stop_logging();
    void set_response_cmd_connect();
    void set_response_cmd_disconnect();

public:
    ExtCommModule() {
        outer_control_state_limited_.state_code.node_id = 3;
        outer_control_state_limited_.state_code.data_size =
            sizeof(ControlStateLimited);
        control_state_limited_ =
            (ControlStateLimited*)outer_control_state_limited_.data;

        cmd_stack_to_external_system_ =
            std::make_shared<NodeCmdStack>(MAX_NODE_CMD_STACK_SIZE);
    };
    ~ExtCommModule() {};
    bool initialize(std::shared_ptr<NodeCmdStack> sys_cmd_stack,
                    std::shared_ptr<NodeStateStack> control_state_stack,
                    node_state* outer_system_state,
                    node_state* outer_control_state,
                    SystemState* inner_system_state, CommUdp* udp) {
        sys_cmd_stack_ = sys_cmd_stack;
        control_state_stack_ = control_state_stack;  // 外部通信用
        outer_system_state_ = outer_system_state;    // 外部通信用
        outer_control_state_ = outer_control_state;  // 外部通信用
        inner_system_state_ = inner_system_state;
        udp_ = udp;
        return true;
    }

    std::shared_ptr<NodeCmdStack> get_cmd_stack_to_external_system_ptr();

    bool recv();
    void send();
};
