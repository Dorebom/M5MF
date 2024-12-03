// clang-format off
#include "main.hpp"
// clang-format on

#include <EthernetUdp.h>  // for LAN
#include <M5_Ethernet.h>  // for LAN
// #include <M5Module_LAN.h>  // for LAN
#include <SPI.h>  // for LAN

#include "Device/comm/can_driver.hpp"
#include "Device/comm/udp_driver.hpp"
//
#include "Control/control_manager.hpp"
#include "System/system_manager.hpp"

// thread priority
#define MAIN_SYSTEM_TASK_PRIORITY 12  // 3
#define COMM_RECV_TASK_PRIORITY   10  // 1
#define CTRL_TASK_PRIORITY        20  // 2
// thread time interval
#define CTRL_TASK_TIME_INTERVAL      100  // 1
#define MAIN_TASK_TIME_INTERVAL      500  // 20
#define COMM_RECV_TASK_TIME_INTERVAL 50
// thread stack size
#define MAIN_STACK_DEPTH      8192
#define CTRL_STACK_DEPTH      8192
#define COMM_RECV_STACK_DEPTH 4096

// Board type
enum class BoardType : int
{
    M5STACK_BASIC = 0,
    M5STACK_CORES3 = 1,
    M5STACK_ATOMS3 = 2
};

BoardType board_type = BoardType::M5STACK_ATOMS3;

/*
 * >> M5STACK PIN DEFINE <<
 */
// ATOMS3R
// M5Stack用ミニCANユニット（TJA1051T/3）使用時
// https://shop.m5stack.com/products/mini-can-unit-tja1051t-3
// ATOMIC W5500搭載PoEベース 使用時
// https://shop.m5stack.com/products/atomic-poe-base-w5500
#define RX_TWAI_NUM 1  // CAN RX pin
#define TX_TWAI_NUM 2  // CAN TX pin
#define LAN_SCK     5  // LAN Module SCK
#define LAN_CS      6  // LAN Module CS
#define LAN_MISO    7  // LAN Module MISO
#define LAN_MOSI    8  // LAN Module MOSI

//  UDP and LAN
#define UDP_PORT_RECV 50001
#define UDP_PORT_SEND 50002
IPAddress destination_ip(192, 168, 8, 116);
IPAddress ip(192, 168, 8, 217);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x99};

/*
 * << END DEFINE >>
 */

SystemManager sys_manager;
ControlManager ctrl_manager;

CommCan can_;
CommUdp udp_;

/*
 * TASK FUNCTIONS
 */
static void comm_receive_task(void *arg) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t watchdog = 0;
    while (!sys_manager.check_initilized_udp_module()) {
        M5_LOGW("Waiting for UDP Module Initialization");
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        watchdog++;
        if (watchdog > 5) {
            M5_LOGE("UDP Module Initialization Failed");
        }
    }
    /*
     * 2. UDP RECEIVE LOOP
     */
    while (sys_manager.check_initilized_udp_module()) {
        sys_manager.recv_from_external_system();
        vTaskDelayUntil(&xLastWakeTime,
                        pdMS_TO_TICKS(COMM_RECV_TASK_TIME_INTERVAL));
        // vTaskDelay(pdMS_TO_TICKS(UDP_RECV_TASK_TIME_INTERVAL - elapsed));
    }
    vTaskDelete(NULL);
}

static void main_task(void *arg) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    /*
     * 1. Initialize
     */
    sys_manager.change_sm_initilizing();
    //
    sys_manager.initialize(&udp_);
    //
    while (!sys_manager.check_initilized_ctrl_task()) {
        M5_LOGW("Waiting for Control Task Initialization");
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
    //
    sys_manager.change_sm_ready();
    //
    /*
     * 2. MAIN LOOP
     */
    while (1) {
        // 1. Get State
        M5_UPDATE();
        // 2. Update
        sys_manager.update();
        M5_LOGI("Main Task Running");
        //
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAIN_TASK_TIME_INTERVAL));
    }
    vTaskDelete(NULL);
}

static void ctrl_task(void *arg) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    /*
     * 1. Initialize
     */
    ctrl_manager.initialize(&can_);
    // >> Set Control Manager Data to System Manager
    sys_manager.set_control_cmd_ptr(ctrl_manager.get_control_cmd_ptr());
    sys_manager.set_control_state_ptr(ctrl_manager.get_control_state_ptr());
    sys_manager.set_initialized_ctrl_task();
    //
    /*
     * 2. CONTROL LOOP
     */
    while (1) {
        // 1. Get State

        // 2. Update
        ctrl_manager.update();
        M5_LOGI("Control Task Running");
        //
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CTRL_TASK_TIME_INTERVAL));
    }
    vTaskDelete(NULL);
}

void setup(void) {
    M5_BEGIN();
    M5_LOGI("M5Stack initialized");

    // 1. Display Logo
    M5.Display.setTextColor(GREEN);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setFont(&fonts::Orbitron_Light_24);
    M5.Display.setTextSize(1);
    M5.Display.drawString("M5MF", M5.Display.width() / 2,
                          M5.Display.height() / 2 - 30);
    M5.Display.drawString("u _ u", M5.Display.width() / 2,
                          M5.Display.height() / 2);
    M5.Display.setTextSize(0.5);
    M5.Display.drawString("Ver. 0.0.1", M5.Display.width() / 2,
                          M5.Display.height() / 2 + 30);

    // 2. Initialize Device
    // >> CAN
    can_.init_twai(TX_TWAI_NUM, RX_TWAI_NUM);
    // >> LAN
    SPI.begin(LAN_SCK, LAN_MISO, LAN_MOSI, -1);  // for LAN
    M5_LOGI("SPI initialized");
    uint8_t cs_pin;
    uint8_t rst_pin;
    uint8_t int_pin;
    switch (board_type) {
        case BoardType::M5STACK_BASIC: {
            cs_pin = 5;    //  5;
            rst_pin = 0;   //  0;
            int_pin = 35;  // 35;
        } break;
        case BoardType::M5STACK_CORES3: {
            cs_pin = 13;
            rst_pin = 0;
            int_pin = 14;
        } break;
        case BoardType::M5STACK_ATOMS3: {
            // TODO: Check the pin number
            cs_pin = 6;
            rst_pin = 0;
            int_pin = 0;
        } break;
        default:
            break;
    }
    M5_LOGI("cs_pin: %d, rst_pin: %d, int_pin: %d", cs_pin, rst_pin, int_pin);

    if (udp_.init_lan(cs_pin, rst_pin, int_pin, mac)) {
        udp_.set_udp_info(ip, destination_ip, UDP_PORT_RECV, UDP_PORT_SEND);
    }

    // 3. Initialize THREAD
    xTaskCreateUniversal(comm_receive_task, "UDP_RECV_TASK",
                         COMM_RECV_STACK_DEPTH, NULL, COMM_RECV_TASK_PRIORITY,
                         NULL, 0);
    xTaskCreateUniversal(main_task, "MAIN_TASK", MAIN_STACK_DEPTH, NULL,
                         MAIN_SYSTEM_TASK_PRIORITY, NULL, 0);
    xTaskCreateUniversal(ctrl_task, "CTRL_TASK", CTRL_STACK_DEPTH, NULL,
                         CTRL_TASK_PRIORITY, NULL, 1);

    M5DEV_LOGI("Thread initialized");
    // <--END 7. Initialize THREAD
}

void loop(void) {
    // M5_UPDATE();
}
