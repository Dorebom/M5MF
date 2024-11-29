// clang-format off
#include "main.hpp"
// clang-format on

#include "Control/control_manager.hpp"
#include "System/system_manager.hpp"

// thread priority
#define MAIN_SYSTEM_TASK_PRIORITY 12  // 3
#define COMM_RECV_TASK_PRIORITY   10  // 1
#define CTRL_TASK_PRIORITY        20  // 2
// thread time interval
#define CTRL_TASK_TIME_INTERVAL      1
#define MAIN_TASK_TIME_INTERVAL      15
#define COMM_RECV_TASK_TIME_INTERVAL 50
// thread stack size
#define MAIN_STACK_DEPTH      8192
#define CTRL_STACK_DEPTH      8192
#define COMM_RECV_STACK_DEPTH 4096

SystemManager sys_manager;
ControlManager ctrl_manager;

/*
 * TASK FUNCTIONS
 */
static void comm_receive_task(void *arg) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    /*
     * 2. UDP RECEIVE LOOP
     */
    while (1) {
        vTaskDelayUntil(&xLastWakeTime,
                        pdMS_TO_TICKS(COMM_RECV_TASK_TIME_INTERVAL));
        // vTaskDelay(pdMS_TO_TICKS(UDP_RECV_TASK_TIME_INTERVAL - elapsed));
    }
    vTaskDelete(NULL);
}

static void main_task(void *arg) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    sys_manager.initialize();
    /*
     * 2. MAIN LOOP
     */
    while (1) {
        // 1. Get State
        M5_UPDATE();

        // 2. Update
        sys_manager.update();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAIN_TASK_TIME_INTERVAL));
        // vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_TIME_INTERVAL - elapsed));
    }
    vTaskDelete(NULL);
}

static void ctrl_task(void *arg) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    ctrl_manager.initialize();
    /*
     * 2. CONTROL LOOP
     */
    while (1) {
        // 1. Get State

        // 2. Update
        ctrl_manager.update();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CTRL_TASK_TIME_INTERVAL));
        // vTaskDelay(pdMS_TO_TICKS(CTRL_TASK_TIME_INTERVAL - elapsed));
    }
    vTaskDelete(NULL);
}

void setup(void) {
    M5_BEGIN();
    M5_LOGI("M5Stack initialized");

    // 7. Initialize THREAD
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
    M5_UPDATE();
}
