#pragma once

#include "esp_err.h"
#include "esp_log.h"

// Identifier for the RollerCAN master.
#define ROLLERCAN_MASTER_ID  0x00
#define ROLLERCAN_DEFAULT_ID 0xA8

/*
 * Define the command ID for the M5stack RollerCan motor driver
 */

// CAN
#define ROLLERCAN_DEVICE_ID     0x00
#define ROLLERCAN_FEEDBACK_DATA 0x02

#define ROLLERCAN_OBTAIN_DEVICE_ID       0x00  // -> 0x00
#define ROLLERCAN_ENABLE                 0x03  // -> 0x02
#define ROLLERCAN_STOP                   0x04  // -> 0x02
#define ROLLERCAN_SET_CAN_ID             0x07  // -> 0x00
#define ROLLERCAN_CLEAR_STALL_PROTECTION 0x09  // -> 0x02
#define ROLLERCAN_SAVE_PARAM             0x0A  // -> 0x02
#define ROLLERCAN_SET_BAUDRATE           0x0B  // -> 0x0B
#define ROLLERCAN_STALL_PROTECTION_ON    0x0C  // -> 0x02
#define ROLLERCAN_STALL_PROTECTION_OFF   0x0D  // -> 0x02
#define ROLLERCAN_READ_PARAM             0x11  // -> 0x11
#define ROLLERCAN_WRITE_PARAM            0x12  // -> 0x02
// I2C
#define ROLLERCAN_READ_I2C_SLAVE_PARAM  0x13  // -> 0x13
#define ROLLERCAN_WRITE_I2C_SLAVE_PARAM 0x14  // -> 0x02
#define ROLLERCAN_READ_I2C_RAW          0x15  // -> 0x15
#define ROLLERCAN_WRITE_I2C_RAW         0x16  // -> 0x16

/*
 * Define the address for the M5stack RollerCan motor driver
 */

// Parameter
#define ROLLERCAN_SAVE_FLASH           0x7002  // uint 8_t // W
#define ROLLERCAN_STALL_PROTECTION     0x7003  // uint 8_t // W
#define ROLLERCAN_ENABLE_SWITCH        0x7004  // uint 8_t // W/R
#define ROLLERCAN_RUN_MODE             0x7005  // uint 8_t // W/R
#define ROLLERCAN_CURRENT              0x7006  //  int32_t // W/R : 0.01 mA
#define ROLLERCAN_SPEED                0x700A  //  int32_t // W/R : 0.01 rpm
#define ROLLERCAN_POSITION             0x7016  //  int32_t // W/R : 0.01 deg
#define ROLLERCAN_POS_MODE_MAX_CURRENT 0x7017  //  int32_t // W/R : 0.01 mA
#define ROLLERCAN_SPD_MODE_MAX_CURRENT 0x7018  //  int32_t // W/R : 0.01 mA
#define ROLLERCAN_SPD_MODE_KP          0x7020  // uint32_t // W/R : 0.00001
#define ROLLERCAN_SPD_MODE_KI          0x7021  // uint32_t // W/R : 0.0000001
#define ROLLERCAN_SPD_MODE_KD          0x7022  // uint32_t // W/R : 0.00001
#define ROLLERCAN_POS_MODE_KP          0x7023  // uint32_t // W/R : 0.00001
#define ROLLERCAN_POS_MODE_KI          0x7024  // uint32_t // W/R : 0.0000001
#define ROLLERCAN_POS_MODE_KD          0x7025  // uint32_t // W/R : 0.00001
#define ROLLERCAN_SPD_READBACK         0x7030  //  int32_t //   R : 0.01 rpm
#define ROLLERCAN_POS_READBACK         0x7031  //  int32_t //   R : 0.01 deg
#define ROLLERCAN_CURRENT_READBACK     0x7032  //  int32_t //   R : 0.01 mA
#define ROLLERCAN_ENCODER_COUNT        0x7033  //  int32_t // W/R : count
#define ROLLERCAN_VOLTAGE              0x7034  //  int32_t //   R : 0.01 V
#define ROLLERCAN_TEMPERATURE          0x7035  //  int32_t //   R
#define ROLLERCAN_RGB_MODE             0x7050  // uint 8_t // W/R
#define ROLLERCAN_RGB_COLOR            0x7051  // uint32_t // W/R
#define ROLLERCAN_RGB_BRIGHTNESS       0x7052  // uint 8_t // W/R : 0 - 100

/*
 * Define the run mode for the M5stack RollerCan motor driver
 */
#define ROLLERCAN_MODE_SPEED    0x01
#define ROLLERCAN_MODE_POSITION 0x02
#define ROLLERCAN_MODE_CURRENT  0x03
#define ROLLERCAN_MODE_ENCODER  0x04

#define ROLLERCAN_RGB_MODE_DEFAULT 0x00
#define ROLLERCAN_RGB_MODE_CUSTUM  0x01

#define ROLLERCAN_SW_ENABLE  0x01
#define ROLLERCAN_SW_DISABLE 0x00

/*
 * Define the motor thresholds for the M5stack RollerCan motor driver
 */

// Referance
#define ROLLERCAN_CUR_MAX 120000   // 0.01mA
#define ROLLERCAN_CUR_MIN -120000  // 0.01mA

#define ROLLERCAN_RGB_BRIGHTNESS_MAX 100
#define ROLLERCAN_RGB_BRIGHTNESS_MIN 0