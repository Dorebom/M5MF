#pragma once

#include <cstdint>
#include <cstring>

#include "Control/st_control_state.hpp"
#include "driver/twai.h"
#include "force_sensor_def.hpp"

// 主軸感度
#define FORCE_SENSITIVITY  32.765
#define TORQUE_SENSITIVITY 436.867

#define FORCE_ZERO_VALUE  8192
#define TORQUE_ZERO_VALUE 8192

struct ForceSensorState
{
    double fx;
    double fy;
    double fz;
    double mx;
    double my;
    double mz;
    //
    uint16_t fx_raw;
    uint16_t fy_raw;
    uint16_t fz_raw;
    uint16_t mx_raw;
    uint16_t my_raw;
    uint16_t mz_raw;
    uint16_t temperature;
    //
    uint32_t timestamp;
    uint8_t can_id;
};

class CanForceSensor {
private:
    /* data */
    uint8_t master_can_id;

    void generate_twai_message(twai_message_t &msg, uint8_t can_id,
                               uint8_t cmd_id, uint16_t option,
                               uint8_t data_length, uint8_t *data) {
        uint32_t id = cmd_id << 24 | option << 8 | can_id;

        // msg.extd = id;
        msg.flags = TWAI_MSG_FLAG_EXTD;
        msg.extd = 1;
        msg.identifier = id;
        msg.data_length_code = data_length;
        for (int i = 0; i < data_length; i++) {
            msg.data[i] = data[i];
        }
    }

public:
    CanForceSensor(/* args */) {
    }
    ~CanForceSensor() {
    }

    void start(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_START, master_can_id, 8, data);
    }

    void stop(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_STOP, master_can_id, 8, data);
    }

    void get_force_state(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_RESPONSE_FORCE_STATE,
                              master_can_id, 8, data);
    }

    void get_torque_state(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_RESPONSE_TORQUE_STATE,
                              master_can_id, 8, data);
    }

    bool update_force_sensor_state(uint8_t can_id, twai_message_t &msg,
                                   ForceSensorState &state_) {
        state_.fx_raw = msg.data[1] | msg.data[0] << 8;
        state_.fy_raw = msg.data[3] | msg.data[2] << 8;
        state_.fz_raw = msg.data[5] | msg.data[4] << 8;

        state_.fx = (state_.fx_raw - FORCE_ZERO_VALUE) / FORCE_SENSITIVITY;
        state_.fy = (state_.fy_raw - FORCE_ZERO_VALUE) / FORCE_SENSITIVITY;
        state_.fz = (state_.fz_raw - FORCE_ZERO_VALUE) / FORCE_SENSITIVITY;

        state_.timestamp = micros();
        return true;
    }

    bool update_torque_sensor_state(uint8_t can_id, twai_message_t &msg,
                                    ForceSensorState &state_) {
        state_.mx_raw = msg.data[1] | msg.data[0] << 8;
        state_.my_raw = msg.data[3] | msg.data[2] << 8;
        state_.mz_raw = msg.data[5] | msg.data[4] << 8;

        state_.mx = (state_.mx_raw - TORQUE_ZERO_VALUE) / TORQUE_SENSITIVITY;
        state_.my = (state_.my_raw - TORQUE_ZERO_VALUE) / TORQUE_SENSITIVITY;
        state_.mz = (state_.mz_raw - TORQUE_ZERO_VALUE) / TORQUE_SENSITIVITY;

        state_.timestamp = micros();
        return true;
    }

    void update_status(twai_message_t &rx_msg, ForceSensorState &state) {
        // 受信データの解析
        uint8_t can_id = (rx_msg.identifier & 0x0000FF00) >> 8;
        uint8_t msg_cmd = (rx_msg.identifier & 0xFF000000) >> 24;

        if (can_id != state.can_id) {
            return;
        }

        switch (msg_cmd) {
            case CMD_RESPONSE_FORCE_STATE:
                update_force_sensor_state(can_id, rx_msg, state);
                break;
            case CMD_RESPONSE_TORQUE_STATE:
                update_torque_sensor_state(can_id, rx_msg, state);
                break;
            default:
                break;
        }
    }
};