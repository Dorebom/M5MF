#pragma once

#include <M5Unified.h>

class ForceStopTactile {
private:
    /* data */
    bool is_force_stop_ = false;
    bool is_processing = false;

public:
    ForceStopTactile(/* args */) {
    }
    ~ForceStopTactile() {
    }
    // >> Set Force Stop
    bool check_force_stop() {
        if (M5.BtnA.wasPressed() && is_force_stop_ == false &&
            is_processing == false) {
            is_force_stop_ = true;
            is_processing = true;
        } else if (M5.BtnA.pressedFor(1000) && is_force_stop_ == true &&
                   is_processing == false) {
            is_force_stop_ = false;
            is_processing = true;
        } else if (M5.BtnA.wasReleased() && is_processing == true) {
            is_processing = false;
        }
        return is_force_stop_;
    }
};