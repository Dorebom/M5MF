#pragma once

#include <string>

#include "Common/node_cmd.hpp"

struct SystemState
{
    uint32_t ave_calc_time_of_main_task;
    uint32_t max_calc_time_of_main_task;
    uint32_t ave_calc_time_of_ctrl_task;
    uint32_t max_calc_time_of_ctrl_task;
    uint32_t ave_calc_time_of_udp_send_task;
    uint32_t max_calc_time_of_udp_send_task;
    bool is_manual_force_stop;
    bool is_connected_udp;
    bool is_occured_error_;
    bool is_occured_warning_;
    bool is_streaming_state;
    bool is_logging;
    bool is_requested_state_at_once;
    bool is_init_ctrl_task;
    bool is_init_lan;
    bool dummmy[7];
    unsigned short int udp_recv_num;
    unsigned short int udp_send_num;
    signed short int dummy2[1];
    M5MF_CMD_LIST act_cmd_type;
    int32_t dummy3;

    SystemState() {
        init();
    }

    void init() {
        is_connected_udp = false;
        is_streaming_state = false;
        is_logging = false;
        is_requested_state_at_once = false;
        is_occured_error_ = false;
        is_occured_warning_ = false;
        ave_calc_time_of_main_task = 0;
        max_calc_time_of_main_task = 0;
        ave_calc_time_of_ctrl_task = 0;
        max_calc_time_of_ctrl_task = 0;
        ave_calc_time_of_udp_send_task = 0;
        max_calc_time_of_udp_send_task = 0;
        is_manual_force_stop = false;
        udp_recv_num = 0;
        udp_send_num = 0;
        act_cmd_type = M5MF_CMD_LIST::NONE;

        dummy3 = 0xFFFF;
    }
};
