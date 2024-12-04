#pragma once

class ForceStopModule {
private:
    /* data */
    bool is_force_stop_ = false;
    bool is_prev_force_stop = false;
    bool is_manual_force_stop = false;

public:
    ForceStopModule(/* args */) {
    }
    ~ForceStopModule() {
    }
    // >> Set Force Stop
    void set_force_stop() {
        is_prev_force_stop = is_force_stop_;
        is_force_stop_ = true;
    }
    void set_manual_force_stop() {
        is_prev_force_stop = is_force_stop_;
        is_manual_force_stop = true;
        is_force_stop_ = true;
    }
    // >> Release Force Stop
    void release_force_stop() {
        is_prev_force_stop = is_force_stop_;
        if (!is_manual_force_stop) {
            is_force_stop_ = false;
        }
    }
    void release_manual_force_stop() {
        is_prev_force_stop = is_force_stop_;
        is_manual_force_stop = false;
        is_force_stop_ = false;
    }
    // >> Check Force Stop
    bool check_force_stop() {
        return is_force_stop_;
    }
    bool check_force_stop_changed() {
        // if (is_force_stop_ != is_prev_force_stop) {
        if (!is_force_stop_ && is_prev_force_stop) {
            return true;
        }
        return false;
    }
};