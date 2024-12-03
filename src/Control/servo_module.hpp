#pragma once

#include <map>

#include "Device/comm/can_driver.hpp"

enum class CanServoType
{
    M5ROLLER,
    CYBERGEAR,
};

struct ServoTypeInfo
{
    /* data */
    uint8_t can_id;
    CanServoType type;
};

class ServoModule {
private:
    std::map<uint8_t, ServoTypeInfo> servo_info_map_;
    CommCan* can_driver_;

public:
    ServoModule(/* args */);
    ~ServoModule();

    void init(CommCan* can_driver);
};
