#pragma once

#include "Service/stacker.hpp"
#include "m5mf_cmd_list.hpp"
#include "node_state_machine.hpp"

#define MAX_NODE_CMD_DATA_SIZE  200
#define MAX_NODE_CMD_STACK_SIZE 10

struct common_cmd_code  // 32 bytes
{
    /* data */
    int source;
    int destination;
    int priority;
    int cmd_id;
    M5MF_CMD_LIST cmd_type;
    int data_size;
    bool is_sys_cmd;
    bool is_used_msgpack;
    uint16_t dummy1;
    uint32_t dummy2;

    common_cmd_code(/* args */) {
        source = 0;
        destination = 0;
        priority = 0;
        cmd_id = 0;
        cmd_type = M5MF_CMD_LIST::NONE;
        data_size = 0;
        is_sys_cmd = false;
        is_used_msgpack = false;
        dummy1 = 0;
        dummy2 = 0;
    }
};

struct st_node_cmd
{
    /* data */
    common_cmd_code cmd_code;
    std::uint8_t data[MAX_NODE_CMD_DATA_SIZE];
    st_node_cmd(/* args */) {
        cmd_code = common_cmd_code();
    }
    void default_init() {
        cmd_code.is_used_msgpack = false;
        cmd_code.is_sys_cmd = false;
        cmd_code.priority = 0;
        cmd_code.source = 0;
        cmd_code.data_size = 0;
        cmd_code.cmd_id = 0;
    }
};

class NodeCmdStack {
private:
    /* data */
public:
    circular_stacker<st_node_cmd> cmd_stack_;
    NodeCmdStack(size_t stack_size) : cmd_stack_(stack_size) {
        cmd_stack_.set_name("NodeCmdStack");
    }
    ~NodeCmdStack() {
    }
};