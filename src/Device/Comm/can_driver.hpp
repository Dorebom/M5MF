#pragma once

#include "driver/twai.h"

#define CAN_WAITNG_TIME 10
class CommCan {
private:
public:
    CommCan(/* args */);
    ~CommCan();

    void init_twai(uint8_t tx_num, uint8_t rx_num);
    void send_can_packet_task(const twai_message_t& packet);
    bool recv_can_packet_task(twai_message_t& packet);
    bool callback(twai_message_t& send_packet, twai_message_t& recv_packet);
};
