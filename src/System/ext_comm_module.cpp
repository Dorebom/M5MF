#include "ext_comm_module.hpp"

#include "Service/logger.hpp"

int ExtCommModule::generate_send_packet() {
    // (機能)ext_send_packet_を生成する

    if (inner_system_state_->is_logging) {
        auto state_stack_size = control_state_stack_->state_stack_.size();

        for (int i = 0; i < state_stack_size; i++) {
            auto temp_state_data = control_state_stack_->state_stack_.pop();
            control_state_limited_->compressed_copy(
                *(ControlState*)temp_state_data.data);
            set_udp_send_state(&outer_control_state_limited_);
        }
    }
    if (inner_system_state_->is_streaming_state) {
        set_udp_send_state(outer_control_state_);
    } else if (inner_system_state_->is_requested_state_at_once) {
        set_udp_send_state(outer_control_state_);
        inner_system_state_->is_requested_state_at_once = false;
    }

    if (!is_prev_logging && inner_system_state_->is_logging) {
        set_response_cmd_start_logging();
    } else if (is_prev_logging && !inner_system_state_->is_logging) {
        set_response_cmd_stop_logging();
    }

    // パケットサイズを計算
    if (ext_send_packet_.stack_marker_num > 0) {
        int packet_size =
            ext_send_packet_.stack_num * ext_send_packet_.one_stack_size +
            ext_send_packet_.data_frame_header_size;
        return packet_size;
    }
    return 0;
}

void ExtCommModule::set_udp_send_state(st_node_state* state) {
    int stack_marker_size;
    int stack_data_size =
        state->state_code.data_size + sizeof(common_state_code);
    //
    if (stack_data_size > ext_send_packet_.max_stack_size_at_once) {
        M5_LOGE("(STATE)Stack Data Size Over %d", stack_data_size);
        return;
    }
    //
    int stack_data_surplus = stack_data_size % ext_send_packet_.one_stack_size;
    if (stack_data_surplus == 0) {
        stack_marker_size = stack_data_size / ext_send_packet_.one_stack_size;
    } else {
        stack_marker_size = (stack_data_size - stack_data_surplus) /
                                ext_send_packet_.one_stack_size +
                            1;
    }
    //
    if (stack_marker_size >
        ext_send_packet_.max_stack_marker_size - ext_send_packet_.stack_num) {
        is_unsent_data = true;
        memcpy(unsent_data, state, stack_data_size);
        unsent_stack_marker = stack_marker_size;
        return;
    } else {
        memcpy(&ext_send_packet_.data[ext_send_packet_.stack_num *
                                      ext_send_packet_.one_stack_size],
               state, stack_data_size);
        ext_send_packet_.stack_marker[ext_send_packet_.stack_marker_num] =
            stack_marker_size;
        // stateの場合は、プラスをつける
        ext_send_packet_.stack_num += stack_marker_size;
        ext_send_packet_.stack_marker_num++;
    }
}

void ExtCommModule::set_udp_send_cmd(st_node_cmd* cmd) {
    int stack_marker_size;
    int stack_data_size = cmd->cmd_code.data_size + sizeof(common_cmd_code);
    //
    if (stack_data_size > ext_send_packet_.max_stack_size_at_once) {
        M5_LOGE("(CMD)Stack Data Size Over %d", stack_data_size);
        return;
    }
    //
    int stack_data_surplus = stack_data_size % ext_send_packet_.one_stack_size;
    if (stack_data_surplus == 0) {
        stack_marker_size = stack_data_size / ext_send_packet_.one_stack_size;
    } else {
        stack_marker_size = (stack_data_size - stack_data_surplus) /
                                ext_send_packet_.one_stack_size +
                            1;
    }
    //
    if (stack_marker_size >
        ext_send_packet_.max_stack_marker_size - ext_send_packet_.stack_num) {
        M5_LOGW("(CMD)Stack Marker Size Over (marker size)%d, (recent_sum)%d",
                stack_marker_size, ext_send_packet_.stack_num);
        is_unsent_data = true;
        memcpy(unsent_data, cmd, stack_data_size);
        unsent_stack_marker = -stack_marker_size;
        return;
    } else {
        memcpy(&ext_send_packet_.data[ext_send_packet_.stack_num *
                                      ext_send_packet_.one_stack_size],
               cmd, stack_data_size);
        ext_send_packet_.stack_marker[ext_send_packet_.stack_marker_num] =
            -stack_marker_size;
        // commandの場合は、マイナスをつける
        ext_send_packet_.stack_num += stack_marker_size;
        ext_send_packet_.stack_marker_num++;
    }
}

void ExtCommModule::reset_udp_send_packet(bool discard_unsent_data) {
    ext_send_packet_.stack_num = 0;
    //
    ext_send_packet_.stack_marker_num = 0;
    memset(ext_send_packet_.stack_marker, 0, MAX_UDP_STACK_MARKER_NUM);
    // memset(ext_send_packet_.data, 0, MAX_UDP_SEND_STATE_DATA_SIZE);
    if (is_unsent_data && !discard_unsent_data) {
        memcpy(&ext_send_packet_.data[0], unsent_data,
               std::abs(unsent_stack_marker) * ext_send_packet_.one_stack_size);
        ext_send_packet_.stack_marker[0] = unsent_stack_marker;
        ext_send_packet_.stack_num += std::abs(unsent_stack_marker);
        ext_send_packet_.stack_marker_num++;
        is_unsent_data = false;
        M5_LOGI("Unsent Data Sent");
    }
}

void ExtCommModule::set_response_cmd_start_logging() {
    st_node_cmd res_cmd;
    res_cmd.default_init();
    res_cmd.cmd_code.cmd_type = M5MF_CMD_LIST::START_LOGGING;
    set_udp_send_cmd(&res_cmd);
}

void ExtCommModule::set_response_cmd_stop_logging() {
    st_node_cmd res_cmd;
    res_cmd.default_init();
    res_cmd.cmd_code.cmd_type = M5MF_CMD_LIST::STOP_LOGGING;
    set_udp_send_cmd(&res_cmd);
}

bool ExtCommModule::recv() {
    auto packet_size = udp_->recv_packet_task(recv_packet_buffer);
    if (packet_size > 0) {
        sys_cmd_stack_->cmd_stack_.push(*(st_node_cmd*)recv_packet_buffer);
        return true;
    }
    return false;
}

void ExtCommModule::send() {
    int send_packet_size = generate_send_packet();
    memcpy(send_packet_buffer, &ext_send_packet_, send_packet_size);
    udp_->send_packet_task(send_packet_buffer, send_packet_size);
    is_prev_logging = inner_system_state_->is_logging;
}
