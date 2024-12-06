#pragma once

#include "Service/logger.hpp"
#include "driver/twai.h"
#include "m5roller_def.hpp"

struct M5rollerDriverStatus
{
    /* data */
    uint32_t timestamp;

    int32_t pos_raw;  // deg
    int32_t spd_raw;  // rpm
    int32_t cur_raw;  // mA
    int32_t vlt_raw;  // V

    float pos_ref;
    float vel_ref;
    float spd_ref;

    float act_pos;
    float act_vel;
    float act_cur;

    float pos_kp;
    float pos_ki;
    float pos_kd;
    float spd_kp;
    float spd_ki;
    float spd_kd;

    float limit_cur_in_pos_mode;
    float limit_cur_in_spd_mode;

    float spd_readback;
    float pos_readback;
    float cur_readback;

    float encoder_count;
    float input_voltage;
    float temperature;

    uint8_t rgb_mode;
    uint8_t rgb_color_r;
    uint8_t rgb_color_g;
    uint8_t rgb_color_b;
    uint8_t rgb_brightness;

    uint8_t can_id;
    uint8_t run_mode;
    uint8_t mcu_status;
    uint8_t fault_info;

    uint8_t dummy1;

    bool is_enable;
    bool is_clear_stall_protection;
};

class M5RollerDriver {
private:
    uint8_t master_can_id;
    double rpm_to_rad_per_sec;
    double rad_per_sec_to_rpm;
    double deg_to_rad;
    double rad_to_deg;

    void generate_twai_message(twai_message_t &msg, uint8_t can_id,
                               uint8_t cmd_id, uint16_t option,
                               uint8_t data_length, uint8_t *data) {
        uint32_t id = 0x00000000 | cmd_id << 24 | option << 8 | can_id;

        // msg.extd = id;
        msg.flags = TWAI_MSG_FLAG_EXTD;
        msg.extd = 1;
        msg.identifier = id;
        msg.data_length_code = data_length;
        for (int i = 0; i < data_length; i++) {
            msg.data[i] = data[i];
        }

        /*
        M5_LOGI("generate_twai_message: %d %d %d %d %d %d %d %d %d %d",
                msg.flags, msg.extd, msg.identifier, msg.data_length_code,
                msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
                msg.data[5]);
                */
    }

public:
    M5RollerDriver() : master_can_id(ROLLERCAN_MASTER_ID) {
        rpm_to_rad_per_sec = 2 * M_PI / 60;
        rad_per_sec_to_rpm = 60 / (2 * M_PI);
        deg_to_rad = M_PI / 180;
        rad_to_deg = 180 / M_PI;
    }
    // 2.1 Obtain Device ID
    // Requests a specified ID to check if the device communication is
    // functioning correctly.
    void check_device_id(uint8_t can_id, uint8_t new_can_id,
                         twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, ROLLERCAN_OBTAIN_DEVICE_ID,
                              master_can_id, 8, data);
    }

    // 2.2 Motor Feedback Data
    // Skip

    // 2.3 Motor Enable Operation
    // Enables the motor and puts it into operation.
    // This command starts the motor and makes it enter a preset operation mode.
    void enable_motor(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, ROLLERCAN_ENABLE, master_can_id, 8,
                              data);
    }

    // 2.4 Motor Stop Operation
    // Stops the motor's operation.
    // Sending this command immediately halts the motor.
    void stop_motor(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, ROLLERCAN_STOP, master_can_id, 8,
                              data);
    }

    // 2.5 Set Motor CAN ID
    // Sets the motor's CAN communication ID.
    // This command assigns a unique communication identifier to the motor.
    void set_motor_id(uint8_t can_id, uint8_t new_can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        uint16_t option = new_can_id << 8 | master_can_id;
        generate_twai_message(msg, can_id, ROLLERCAN_SET_CAN_ID, option, 8,
                              data);
    }

    // 2.6 Disable Motor Stall Protection
    // Releases the motor from stall protection mode.
    // This command clears the stall lock after a stall, allowing the motor to
    // resume operation. [NOTE] To operate without stall protection, use the
    // Disable Motor Stall Protection command 2.10.
    void clear_stall_protection(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, ROLLERCAN_CLEAR_STALL_PROTECTION,
                              master_can_id, 8, data);
    }

    // 2.7 Save Parameters to Flash
    // Saves the current parameters to Flash memory.
    // This command permanently stores the current motor settings inside the
    // device.
    void save_param_to_flash(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, ROLLERCAN_SAVE_PARAM, master_can_id,
                              8, data);
    }

    // 2.8 Set Motor Baud Rate
    // Sets the motor's communication baud rate.
    // This command changes the motor's communication baud rate.
    void set_baudrate(uint8_t can_id, uint8_t baudrate, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        uint16_t option = baudrate << 8 | master_can_id;
        generate_twai_message(msg, can_id, ROLLERCAN_SET_BAUDRATE, option, 8,
                              data);
    }

    // 2.9 Enable Motor Stall Protection
    // Enables the motor's stall protection function.
    // This function automatically detects stalls and protects the motor from
    // damage.
    void enable_stall_protection(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, ROLLERCAN_STALL_PROTECTION_ON,
                              master_can_id, 8, data);
    }

    // 2.10 Disable Motor Stall Protection
    // Disables the motor's stall protection function.
    // This function disables the stall protection function, allowing the motor
    // to operate without stall protection.
    void disable_stall_protection(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, ROLLERCAN_STALL_PROTECTION_OFF,
                              master_can_id, 8, data);
    }

    // 2.11 Read Motor Parameters
    // Reads the motor's related parameter information.
    // This command returns real-time data such as motor speed and position.
    void read_motor_param(uint8_t can_id, uint16_t param_index,
                          twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        data[0] = param_index & 0xFF;
        data[1] = (param_index >> 8) & 0xFF;
        generate_twai_message(msg, can_id, ROLLERCAN_READ_PARAM, master_can_id,
                              8, data);
    }

    // 2.11.7004 read_motor_enable_switch
    void read_motor_enable_switch(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_ENABLE_SWITCH, msg);
    }

    // 2.11.7005 read_run_mode
    void read_run_mode(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_RUN_MODE, msg);
    }

    // 2.11.7006 read_current
    void read_current(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_CURRENT, msg);
    }

    // 2.11.700A read_speed
    void read_speed(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_SPEED, msg);
    }

    // 2.11.7016 read_position
    void read_position(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_POSITION, msg);
    }

    // 2.11.7017 read_threshold_position_mode_current
    void read_threshold_position_mode_current(uint8_t can_id,
                                              twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_POS_MODE_MAX_CURRENT, msg);
    }

    // 2.11.7018 read_threshold_speed_mode_current
    void read_threshold_speed_mode_current(uint8_t can_id,
                                           twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_SPD_MODE_MAX_CURRENT, msg);
    }

    // 2.11.7020 read_speed_mode_kp
    void read_speed_mode_kp(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_SPD_MODE_KP, msg);
    }

    // 2.11.7021 read_speed_mode_ki
    void read_speed_mode_ki(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_SPD_MODE_KI, msg);
    }

    // 2.11.7022 read_speed_mode_kd
    void read_speed_mode_kd(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_SPD_MODE_KD, msg);
    }

    // 2.11.7023 read_position_mode_kp
    void read_position_mode_kp(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_POS_MODE_KP, msg);
    }

    // 2.11.7024 read_position_mode_ki
    void read_position_mode_ki(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_POS_MODE_KI, msg);
    }

    // 2.11.7025 read_position_mode_kd
    void read_position_mode_kd(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_POS_MODE_KD, msg);
    }

    // 2.11.7030 read_speed_readback
    void read_speed_readback(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_SPD_READBACK, msg);
    }

    // 2.11.7031 read_position_readback
    void read_position_readback(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_POS_READBACK, msg);
    }

    // 2.11.7032 read_current_readback
    void read_current_readback(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_CURRENT_READBACK, msg);
    }

    // 2.11.7033 read_encoder_count
    void read_encoder_count(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_ENCODER_COUNT, msg);
    }

    // 2.11.7034 read_inputZ_voltage
    void read_input_voltage(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_VOLTAGE, msg);
    }

    // 2.11.7035 read_temperature
    void read_temperature(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_TEMPERATURE, msg);
    }

    // 2.11.7050 read_rgb_mode
    void read_rgb_mode(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_RGB_MODE, msg);
    }

    // 2.11.7051 read_rgb_color
    void read_rgb_color(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_RGB_COLOR, msg);
    }

    // 2.11.7052 read_rgb_brightness
    void read_rgb_brightness(uint8_t can_id, twai_message_t &msg) {
        read_motor_param(can_id, ROLLERCAN_RGB_BRIGHTNESS, msg);
    }

    // 2.12 Write Motor Parameters
    // Writes the set parameters tothe motor.
    // This command allows you to change the motor's working parameters.
    void write_motor_param(uint8_t can_id, uint16_t param_index, uint32_t value,
                           twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        data[0] = param_index & 0xFF;
        data[1] = (param_index >> 8) & 0xFF;
        data[4] = value & 0xFF;
        data[5] = (value >> 8) & 0xFF;
        data[6] = (value >> 16) & 0xFF;
        data[7] = (value >> 24) & 0xFF;
        generate_twai_message(msg, can_id, ROLLERCAN_WRITE_PARAM, master_can_id,
                              8, data);
    }

    // 2.12.7005 change_run_mode
    void change_run_mode(uint8_t can_id, uint8_t mode, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_RUN_MODE, mode, msg);
    }

    // 2.12.7017 set_threshold_position_mode_current
    void set_threshold_position_mode_current(uint8_t can_id,
                                             int32_t max_current_value,
                                             twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_POS_MODE_MAX_CURRENT,
                          max_current_value, msg);
    }

    // 2.12.7018 set_threshold_speed_mode_current
    void set_threshold_speed_mode_current(uint8_t can_id,
                                          int32_t max_current_value,
                                          twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_SPD_MODE_MAX_CURRENT,
                          max_current_value, msg);
    }

    // 2.12.7020-22 set_speed_mode_kp, ki, kd
    void set_speed_mode_kp(uint8_t can_id, uint32_t kp, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_SPD_MODE_KP, kp, msg);
    }

    void set_speed_mode_ki(uint8_t can_id, uint32_t ki, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_SPD_MODE_KI, ki, msg);
    }

    void set_speed_mode_kd(uint8_t can_id, uint32_t kd, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_SPD_MODE_KD, kd, msg);
    }

    // 2.12.7023-25 set_position_mode_kp, ki, kd
    void set_position_mode_kp(uint8_t can_id, uint32_t kp,
                              twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_POS_MODE_KP, kp, msg);
    }

    void set_position_mode_ki(uint8_t can_id, uint32_t ki,
                              twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_POS_MODE_KI, ki, msg);
    }

    void set_position_mode_kd(uint8_t can_id, uint32_t kd,
                              twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_POS_MODE_KD, kd, msg);
    }

    // 2.12.7033 set_encoder_count
    void set_encoder_count(uint8_t can_id, int32_t count, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_ENCODER_COUNT, count, msg);
    }

    // 2.12.7050 set_rgb_mode
    void set_rgb_mode(uint8_t can_id, uint8_t mode, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_RGB_MODE, mode, msg);
    }

    // 2.12.7050.1 set_rgb_mode_default
    void set_rgb_mode_default(uint8_t can_id, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_RGB_MODE,
                          ROLLERCAN_RGB_MODE_DEFAULT, msg);
    }

    // 2.12.7050.2 set_rgb_mode_custom
    void set_rgb_mode_custom(uint8_t can_id, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_RGB_MODE, ROLLERCAN_RGB_MODE_CUSTUM,
                          msg);
    }

    // 2.12.7051 set_rgb_color
    void set_rgb_color(uint8_t can_id, uint32_t color, twai_message_t &msg) {
        write_motor_param(can_id, ROLLERCAN_RGB_COLOR, color, msg);
    }
    // 2.12.7051.1 set_rgb_color
    void set_rgb_color(uint8_t can_id, uint8_t r, uint8_t g, uint8_t b,
                       twai_message_t &msg) {
        uint32_t color = (r << 16) | (g << 8) | b;
        write_motor_param(can_id, ROLLERCAN_RGB_COLOR, color, msg);
    }

    // 2.12.7052 set_rgb_brightness
    void set_rgb_brightness(uint8_t can_id, uint8_t brightness,
                            twai_message_t &msg) {
        if (brightness > ROLLERCAN_RGB_BRIGHTNESS_MAX) {
            brightness = ROLLERCAN_RGB_BRIGHTNESS_MAX;
        } else if (brightness < ROLLERCAN_RGB_BRIGHTNESS_MIN) {
            brightness = ROLLERCAN_RGB_BRIGHTNESS_MIN;
        }
        write_motor_param(can_id, ROLLERCAN_RGB_BRIGHTNESS, brightness, msg);
    }

    // 3. Receive data
    void receive_data(twai_message_t &msg, uint8_t can_id,
                      M5rollerDriverStatus &status) {
        uint8_t cmd_id = (msg.identifier & 0xFF000000) >> 24;

        M5_LOGI(">>>>>>>>>>>>>>>>> cmd_id: %d", cmd_id);

        switch (cmd_id) {
            case ROLLERCAN_DEVICE_ID:
                update_device_id(msg, status);
                break;
            case ROLLERCAN_FEEDBACK_DATA:
                update_feedback_data(msg, status);
                break;
            case ROLLERCAN_SET_BAUDRATE:
                update_baudrate(msg, status);
                break;
            case ROLLERCAN_READ_PARAM:
                update_motor_param(msg, status);
                break;
            default:
                break;
        }
    }

    // 3.1 Device ID
    void update_device_id(twai_message_t &msg, M5rollerDriverStatus &status) {
        auto ret_fe = msg.identifier & 0x000000FF;
        if (ret_fe == 0xFE) {
            status.can_id = (msg.identifier & 0x0000FF00) >> 8;
        }
    }

    // 3.2 Motor Feedback Data
    void update_feedback_data(twai_message_t &msg,
                              M5rollerDriverStatus &status) {
        status.timestamp = micros();
        status.pos_raw = (msg.data[3] << 8) | msg.data[2];
        status.spd_raw = (msg.data[1] << 8) | msg.data[0];
        status.cur_raw = (msg.data[5] << 8) | msg.data[4];
        status.vlt_raw = (msg.data[7] << 8) | msg.data[6];

        status.act_pos = (float)status.pos_raw / 100.0f * deg_to_rad;
        status.act_vel = (float)status.spd_raw / 100.0f * rpm_to_rad_per_sec;
        status.act_cur = (float)status.cur_raw / 100.0f;

        status.fault_info = (msg.identifier >> 16) & 0x07;  // Bit16~18: fault
        status.run_mode = (msg.identifier >> 19) & 0x07;    // Bit19~21: mode
        status.mcu_status = (msg.identifier >> 22) & 0x03;  // Bit22~23: status
        // recv_motorId = (rx_msg.identifier >> 8) &
        //                0xFF;  // Bit8~Bit15: CAN ID of current motor
    }

    // 3.3 Set Motor Baud Rate
    void update_baudrate(twai_message_t &msg, M5rollerDriverStatus &status) {
        // To be implemented
    }

    // 3.4 Read Motor Parameters
    void update_motor_param(twai_message_t &msg, M5rollerDriverStatus &status) {
        // 1447
        uint16_t index = (msg.data[1] << 8) | msg.data[0];
        uint32_t value = (msg.data[7] << 24) | (msg.data[6] << 16) |
                         (msg.data[5] << 8) | msg.data[4];
        switch (index) {
            case ROLLERCAN_ENABLE_SWITCH:
                if (msg.data[4] == 0x00) {
                    status.is_enable = false;
                } else {
                    status.is_enable = true;
                }
                break;
            case ROLLERCAN_STALL_PROTECTION:
                if (msg.data[4] == 0x00) {
                    status.is_clear_stall_protection = false;
                } else {
                    status.is_clear_stall_protection = true;
                }
                break;
            case ROLLERCAN_RUN_MODE:
                status.run_mode = msg.data[4];
                break;
            case ROLLERCAN_POS_MODE_KP:
                status.pos_kp = (float)value / 100000.0f;
                break;
            case ROLLERCAN_POS_MODE_KI:
                status.pos_ki = (float)value / 10000000.0f;
                break;
            case ROLLERCAN_POS_MODE_KD:
                status.pos_kd = (float)value / 100000.0f;
                break;
            case ROLLERCAN_SPD_MODE_KP:
                status.spd_kp = (float)value / 100000.0f;
                break;
            case ROLLERCAN_SPD_MODE_KI:
                status.spd_ki = (float)value / 10000000.0f;
                break;
            case ROLLERCAN_SPD_MODE_KD:
                status.spd_kd = (float)value / 100000.0f;
                break;
            case ROLLERCAN_POS_MODE_MAX_CURRENT:
                status.limit_cur_in_pos_mode = (float)value / 100.0f;
                break;
            case ROLLERCAN_SPD_MODE_MAX_CURRENT:
                status.limit_cur_in_spd_mode = (float)value / 100.0f;
                break;
            case ROLLERCAN_ENCODER_COUNT:
                status.encoder_count = value;
                break;
            case ROLLERCAN_VOLTAGE:
                status.input_voltage = (float)value / 100.0f;
                break;
            case ROLLERCAN_TEMPERATURE:
                status.temperature = value;
                break;
            case ROLLERCAN_RGB_MODE:
                status.rgb_mode = msg.data[4];
                break;
            case ROLLERCAN_RGB_COLOR:
                status.rgb_color_b = msg.data[4];
                status.rgb_color_g = msg.data[5];
                status.rgb_color_r = msg.data[6];
                break;
            case ROLLERCAN_RGB_BRIGHTNESS:
                status.rgb_brightness = msg.data[4];
                break;
            default:
                break;
        }
    }

    void display_status(M5rollerDriverStatus &status) {
        M5_LOGI("Status:");
        M5_LOGI("  timestamp: %d", status.timestamp);
        M5_LOGI("  pos_raw: %d", status.pos_raw);
        M5_LOGI("  spd_raw: %d", status.spd_raw);
        M5_LOGI("  cur_raw: %d", status.cur_raw);
        M5_LOGI("  vlt_raw: %d", status.vlt_raw);
        // M5_LOGI("  act_pos: %f", status.act_pos);
        // M5_LOGI("  act_vel: %f", status.act_vel);
        // M5_LOGI("  act_cur: %f", status.act_cur);
        // M5_LOGI("  pos_kp: %f", status.pos_kp);
        // M5_LOGI("  pos_ki: %f", status.pos_ki);
        // M5_LOGI("  pos_kd: %f", status.pos_kd);
        // M5_LOGI("  spd_kp: %f", status.spd_kp);
        // M5_LOGI("  spd_ki: %f", status.spd_ki);
        // M5_LOGI("  spd_kd: %f", status.spd_kd);
        // M5_LOGI("  limit_cur_in_pos_mode: %f", status.limit_cur_in_pos_mode);
        // M5_LOGI("  limit_cur_in_spd_mode: %f", status.limit_cur_in_spd_mode);
        // M5_LOGI("  encoder_count: %f", status.encoder_count);
        // M5_LOGI("  input_voltage: %f", status.input_voltage);
        // M5_LOGI("  temperature: %f", status.temperature);
        // M5_LOGI("  rgb_mode: %d", status.rgb_mode);
        // M5_LOGI("  rgb_color_r: %d", status.rgb_color_r);
        // M5_LOGI("  rgb_color_g: %d", status.rgb_color_g);
        // M5_LOGI("  rgb_color_b: %d", status.rgb_color_b);
        // M5_LOGI("  rgb_brightness: %d", status.rgb_brightness);
        M5_LOGI("  can_id: %d", status.can_id);
        M5_LOGI("  run_mode: %d", status.run_mode);
        // M5_LOGI("  mcu_status: %d", status.mcu_status);
        // M5_LOGI("  fault_info: %d", status.fault_info);
        // M5_LOGI("  is_enable: %d", status.is_enable);
        // M5_LOGI("  is_clear_stall_protection: %d",
        //        status.is_clear_stall_protection);
    }
};