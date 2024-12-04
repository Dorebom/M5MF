#pragma once

#define HEARTBEAT_TIME     6    // sec
#define HEARTBEAT_OFF_TIME 0.1  // sec

class HeartbeatModule {
private:
    /* data */
    int time_interval_;
    int cnt_;
    int max_cnt_;
    int off_cnt_;

public:
    HeartbeatModule(/* args */) {
    }
    ~HeartbeatModule() {
    }
    void set_system_time_interval(int time_interval) {
        time_interval_ = time_interval;
        max_cnt_ = 1000 * HEARTBEAT_TIME / time_interval_;
        off_cnt_ = 1000 * HEARTBEAT_OFF_TIME / time_interval_;
    }
    bool update() {
        cnt_++;
        if (cnt_ <= off_cnt_) {
            return false;
        }
        if (cnt_ >= max_cnt_) {
            cnt_ = 0;
            return false;
        }
        return true;
    }
};