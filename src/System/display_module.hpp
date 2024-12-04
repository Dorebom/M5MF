#pragma once

#include <M5GFX.h>
#include <M5Unified.h>

#include "Common/node_state.hpp"

class DisplayModule {
private:
    /* data */
    M5Canvas* canvas_;
    M5GFX* lcd_;

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

    void display(bool is_heartbeat_high, node_state* outer_system_state) {
        // Display
        M5.Display.setTextSize(1);
        M5.Lcd.fillRect(20, 50, 80, 40, BLACK);
        if (is_heartbeat_high) {
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
        } else {
            M5.Display.drawString("- _ -", M5.Display.width() / 2,
                                  M5.Display.height() / 2);
        }
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