#include "can_driver.hpp"

#include "Service/logger.hpp"

CommCan::CommCan() {
}

CommCan::~CommCan() {
}

void CommCan::init_twai(uint8_t tx_num, uint8_t rx_num) {
    gpio_num_t TX_GPIO_NUM = gpio_num_t(tx_num);
    gpio_num_t RX_GPIO_NUM = gpio_num_t(rx_num);

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

void CommCan::send_can_packet_task(const twai_message_t& packet) {
    twai_transmit(&packet, portMAX_DELAY);
}

bool CommCan::recv_can_packet_task(twai_message_t& packet) {
    auto ret = twai_receive(&packet, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        M5_LOGE(
            "ControlManager::recv_can_packet_task: Failed to receive CAN "
            "packet");
        return false;
    }
    return true;
}

bool CommCan::callback(twai_message_t& send_packet,
                       twai_message_t& recv_packet) {
    send_can_packet_task(send_packet);
    return recv_can_packet_task(recv_packet);
}
