#pragma once

#include <EthernetUdp.h>
#include <M5Module_LAN.h>
#include <SPI.h>

#include <string>

#define UDP_SEND_PACKET_MAX_SIZE 1400
#define UDP_RECV_PACKET_MAX_SIZE 1400

class CommUdp {
private:
    M5Module_LAN lan_;
    EthernetUDP udp_;

    // >> UDP Info
    IPAddress local_ip;
    IPAddress destination_ip;
    uint32_t UDP_PORT_RECV;
    uint32_t UDP_PORT_SEND;

    // >> UDP Packet
    uint32_t recv_size;
    uint32_t send_size;
    uint8_t send_packet_buffer[UDP_SEND_PACKET_MAX_SIZE];
    uint8_t recv_packet_buffer[UDP_RECV_PACKET_MAX_SIZE];

public:
    CommUdp(/* args */);
    ~CommUdp();

    void set_recv_size(uint32_t size);
    void set_send_size(uint32_t size);

    bool init_lan(uint8_t cs, uint8_t rst, uint8_t int_pin, byte* mac);
    void set_udp_info(IPAddress local_ip_, IPAddress destination_ip_,
                      uint32_t recv_port_, uint32_t send_port_);
    void set_destination_info(IPAddress destination_ip_, uint32_t send_port_);
    void send_packet_task(const uint8_t* packet, size_t size);
    uint32_t recv_packet_task(uint8_t* packet);
};