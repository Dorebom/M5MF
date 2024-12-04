#pragma once

enum class M5MF_CMD_LIST : int
{
    /*
     * System Manager Command List 0 - 99
     */
    /* State machine cmd */
    NONE,
    CHANGE_SM_STABLE,
    CHANGE_SM_READY,
    CHANGE_SM_REPAIR,
    CHANGE_SM_FORCE_STOP,
    RELEASE_FORCE_STOP,

    /* System manager cmd */
    REQUEST_STATE,
    START_STREAM_STATE,
    STOP_STREAM_STATE,
    SET_NODE_ID,
    SET_CONFIG,

    RESET_ERROR,
    RESET_ALERT,
    // 以下、サーボ制御コマンド
    CONNECT_CAN,
    DISCONNECT_CAN,
    CHANGE_CONTROLLED_SRV_ID,
    CHANGE_SRV_POWER,  // ひとつだけサーボON/OFF
    CHANGE_SRV_CTRLMODE,

    /*
     * Control System Command List 100 - 199
     */
    CHANGE_MF_ENABLE = 100,
    CHANGE_MF_STOP,

    SERVO_POSITION_CONTROL,
    SERVO_VELOCITY_CONTROL,
    SERVO_TORQUE_CONTROL,

    SCARA_POSITION_CONTROL,
    SCARA_VELOCITY_CONTROL,
    SCARA_TORQUE_CONTROL,

    START_LOGGING,
    STOP_LOGGING,

    PERIOD_CMD  // この行は削除しないこと

};
