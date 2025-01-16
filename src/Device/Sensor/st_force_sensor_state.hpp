#pragma once

#include <cstdint>

struct ForceSensorState
{
    uint16_t fx_raw;
    uint16_t fy_raw;
    uint16_t fz_raw;
    uint16_t mx_raw;
    uint16_t my_raw;
    uint16_t mz_raw;
    uint16_t temperature;
};