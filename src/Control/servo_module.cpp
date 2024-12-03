#include "servo_module.hpp"

ServoModule::ServoModule(/* args */) {
}

ServoModule::~ServoModule() {
}

void ServoModule::init(CommCan* can_driver) {
    can_driver_ = can_driver;
}