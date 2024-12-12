#pragma once

#include <M5GFX.h>
#include <M5Unified.h>

#include "Common/node_state.hpp"
#include "Control/st_control_state.hpp"
//
#include "System/st_system_state.hpp"

class DisplayModule {
private:
    /* data */
    M5Canvas* canvas_;
    M5GFX* lcd_;

    // Flag
    bool is_prev_heartbeat_high = false;
    bool is_prev_connected_udp = false;
    bool is_prev_logging = false;
    bool is_prev_streaming_state = false;
    node_state_machine prev_state_machine;
    CTRL_MODE_LIST prev_ctrl_mode;
    MECHANICAL_FRAME_LIST prev_mf_type;

public:
    DisplayModule(/* args */) {
    }
    ~DisplayModule() {
    }
    void initialize() {
        // canvas1_.setPsram(false);
        canvas_->createSprite((int32_t)M5.Lcd.width(),
                              (int32_t)M5.Lcd.height());
        canvas_->setTextScroll(false);
        canvas_->setPaletteColor(1, GREEN);
        canvas_->setTextSize(1);
        // canvas_->setTextColor(GREEN);
        //  canvas_->setTextDatum(middle_center);
        // canvas_->setFont(&fonts::Orbitron_Light_24);
    }

    void display() {
        // Display
        // canvas_->clear();
        M5_LOGW("Display");
        canvas_->setCursor(30, 30);
        // canvas_->startWrite();

        canvas_->printf("o _ o\n");
        // canvas1_.drawString("o _ o", M5.Display.width() / 2,
        //                     M5.Display.height() / 2);

        // canvas_->endWrite();
        canvas_->pushSprite(0, 0);
    }

    void display(bool is_heartbeat_high, node_state* outer_system_state,
                 SystemState* inner_system_state, ControlState* control_state) {
        // Display
        M5.Display.setTextSize(1);
        // >> Face
        if ((is_heartbeat_high && !is_prev_heartbeat_high) ||
            prev_state_machine !=
                outer_system_state->state_code.state_machine) {
            M5.Lcd.fillRect(20, 50, 80, 40, BLACK);
            switch (outer_system_state->state_code.state_machine) {
                case node_state_machine::INITIALIZING:
                    M5.Display.setTextColor(WHITE);
                    M5.Display.drawString("- _ -", M5.Display.width() / 2,
                                          M5.Display.height() / 2);
                    break;
                case node_state_machine::READY:
                    M5.Display.setTextColor(WHITE);
                    M5.Display.drawString("u _ u", M5.Display.width() / 2,
                                          M5.Display.height() / 2);
                    break;
                case node_state_machine::STABLE:
                    M5.Display.setTextColor(GREEN);
                    M5.Display.drawString("o _ o", M5.Display.width() / 2,
                                          M5.Display.height() / 2);
                    break;
                case node_state_machine::REPAIR:
                    M5.Display.setTextColor(ORANGE);
                    M5.Display.drawString("o _ o", M5.Display.width() / 2,
                                          M5.Display.height() / 2);
                    break;
                case node_state_machine::FORCE_STOP:
                    M5.Display.setTextColor(RED);
                    M5.Display.drawString("x _ x", M5.Display.width() / 2,
                                          M5.Display.height() / 2);
                    break;
                default:
                    break;
            }
        } else if (!is_heartbeat_high && is_prev_heartbeat_high) {
            M5.Lcd.fillRect(20, 50, 80, 40, BLACK);
            M5.Display.drawString("- _ -", M5.Display.width() / 2,
                                  M5.Display.height() / 2);
        }
        // >> Connecting State
        if (inner_system_state->is_connected_udp && !is_prev_connected_udp) {
            M5.Display.setTextColor(CYAN);
            M5.Display.drawString("M5MF", M5.Display.width() / 2,
                                  M5.Display.height() / 2 - 30);
            M5.Display.setTextColor(GREEN);
        } else if (!inner_system_state->is_connected_udp &&
                   is_prev_connected_udp) {
            M5.Display.setTextColor(WHITE);
            M5.Display.drawString("M5MF", M5.Display.width() / 2,
                                  M5.Display.height() / 2 - 30);
            M5.Display.setTextColor(GREEN);
        }
        // >> Control System State
        if (prev_mf_type != control_state->state_code.mf_type ||
            prev_ctrl_mode != control_state->state_code.ctrl_mode) {
            M5.Lcd.fillRect(0, 100, 128, 25, BLACK);

            M5.Display.setTextColor(GREEN);
            if (control_state->state_code.mf_type ==
                MECHANICAL_FRAME_LIST::ALLJOINT) {
                M5.Display.drawString("MF-1", M5.Display.width() / 2 - 25,
                                      M5.Display.height() / 2 + 50);
            } else if (control_state->state_code.mf_type ==
                       MECHANICAL_FRAME_LIST::SCARA) {
                M5.Display.drawString("MF-2", M5.Display.width() / 2 - 25,
                                      M5.Display.height() / 2 + 50);
            }

            switch (control_state->state_code.ctrl_mode) {
                case CTRL_MODE_LIST::STAY:
                    M5.Display.drawString("S", M5.Display.width() / 2 + 50,
                                          M5.Display.height() / 2 + 50);
                    break;
                case CTRL_MODE_LIST::POSITION:
                    M5.Display.drawString("P", M5.Display.width() / 2 + 50,
                                          M5.Display.height() / 2 + 50);
                    break;
                case CTRL_MODE_LIST::VELOCITY:
                    M5.Display.drawString("V", M5.Display.width() / 2 + 50,
                                          M5.Display.height() / 2 + 50);
                    break;
                case CTRL_MODE_LIST::TORQUE:
                    M5.Display.drawString("T", M5.Display.width() / 2 + 50,
                                          M5.Display.height() / 2 + 50);
                    break;
                default:
                    break;
            }
        }
        // >> Logging State
        if (inner_system_state->is_logging && !is_prev_logging) {
            M5.Lcd.fillCircle(30, 10, 8, RED);
        } else if (!inner_system_state->is_logging && is_prev_logging) {
            M5.Lcd.fillCircle(30, 10, 8, BLACK);
        }
        // >> Streaming State
        if (inner_system_state->is_streaming_state &&
            !is_prev_streaming_state) {
            M5.Lcd.fillTriangle(3, 3, 3, 17, 13, 10, GREEN);
        } else if (!inner_system_state->is_streaming_state &&
                   is_prev_streaming_state) {
            M5.Lcd.fillTriangle(3, 3, 3, 17, 13, 10, BLACK);
        }

        is_prev_heartbeat_high = is_heartbeat_high;
        is_prev_connected_udp = inner_system_state->is_connected_udp;
        is_prev_logging = inner_system_state->is_logging;
        is_prev_streaming_state = inner_system_state->is_streaming_state;
        prev_state_machine = outer_system_state->state_code.state_machine;
        prev_ctrl_mode = control_state->state_code.ctrl_mode;
        prev_mf_type = control_state->state_code.mf_type;
    }
    void reset_display() {
        M5.Display.setTextColor(GREEN);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setFont(&fonts::Orbitron_Light_24);
        M5.Display.setTextSize(1);
        M5.Display.drawString("M5MF", M5.Display.width() / 2,
                              M5.Display.height() / 2 - 30);
        M5.Display.drawString("u _ u", M5.Display.width() / 2,
                              M5.Display.height() / 2);
        M5.Display.setTextSize(0.5);
        M5.Display.drawString("Ver. 0.0.1", M5.Display.width() / 2,
                              M5.Display.height() / 2 + 30);
    }

    void set_lcd(M5GFX& lcd) {
        lcd_ = &lcd;
    }
    void set_canvas(M5Canvas* canvas) {
        canvas_ = canvas;
        initialize();
    }
};