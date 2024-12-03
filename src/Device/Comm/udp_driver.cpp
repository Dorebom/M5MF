#include "udp_driver.hpp"

#include "Service/logger.hpp"

CommUdp::CommUdp() {
}

CommUdp::~CommUdp() {
}

void CommUdp::set_recv_size(uint32_t size) {
    recv_size = size;
}

void CommUdp::set_send_size(uint32_t size) {
    send_size = size;
}

bool CommUdp::init_lan(uint8_t cs, uint8_t rst, uint8_t int_pin, byte* mac) {
    // Initialize LAN
    M5_LOGI("mac: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2],
            mac[3], mac[4], mac[5]);
    uint8_t watchdog = 0;
    // SPI.begin();
    lan_.setResetPin(rst);
    lan_.reset();
    M5_LOGI("LAN reset");
    lan_.init(cs);
    M5_LOGI("LAN initializing");
    while (lan_.begin(mac, 1000, 1000) != 1) {
        M5DEV_LOGE("Error getting IP address via DHCP, trying again...");
        delay(1000);
        watchdog++;
        if (watchdog > 5) {
            M5DEV_LOGE("Ethernet shield was not found. Break");
            return false;
        }
    }
    M5_LOGI("(STEP 1)Ethernet shield was found.");
    if (watchdog <= 5) {
        watchdog = 0;
        while (lan_.hardwareStatus() == EthernetNoHardware) {
            M5DEV_LOGE(
                "Ethernet shield was not found.  Sorry, can't run without "
                "hardware. :(");
            delay(500);
            watchdog++;
            if (watchdog > 5) {
                M5DEV_LOGE("Ethernet shield was not found. Break");
                return false;
            }
        }
    }
    M5_LOGI("(STEP 2)Ethernet shield was found.");
    if (watchdog <= 5) {
        watchdog = 0;
        while (lan_.linkStatus() == LinkOFF) {
            M5DEV_LOGE("Ethernet cable is not connected.");
            delay(500);
            watchdog++;
            if (watchdog > 5) {
                M5DEV_LOGE("Ethernet cable is not connected. Break");
                return false;
            }
        }
    }
    M5DEV_LOGI("(STEP 3)Ethernet cable is connected.");
    return true;
}

void CommUdp::set_udp_info(IPAddress local_ip_, IPAddress destination_ip_,
                           uint32_t recv_port_, uint32_t send_port_) {
    local_ip = local_ip_;
    destination_ip = destination_ip_;
    UDP_PORT_RECV = recv_port_;
    UDP_PORT_SEND = send_port_;

    udp_.begin(UDP_PORT_RECV);
    M5_LOGI("UDP initialized");
}

void CommUdp::send_packet_task(const uint8_t* packet, size_t size) {
    if (size == 0) {
        return;
    }
    udp_.beginPacket(destination_ip, UDP_PORT_SEND);
    udp_.write(packet, size);
    udp_.endPacket();
}

uint32_t CommUdp::recv_packet_task(uint8_t* packet) {
    int packet_size = udp_.parsePacket();
    if (packet_size > 0) {
        // packet.reserve(packet_size);
        udp_.read(packet, packet_size);
        return packet_size;
    }

    return 0;
}

bool CommUdp::callback(std::string& send_packet, std::string& recv_packet) {
    return false;
}
