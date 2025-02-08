#pragma once

#include <Arduino.h>

// Callback function declarations (defined in main.cpp)
extern void onDispense();
extern void onReverse();
extern void onForward();
extern void onStop();
extern void onOpen();
extern void onClose();
extern void onStepForward();
extern void onStepBackward();
extern void escalatorStart();
extern void escalatorStop();


// Command structure
struct MqttCommand {
    const char* command;
    void (*callback)();
};

// Lookup table for MQTT commands
const MqttCommand mqttCommandLookup[] = {
    {"dispense", onDispense},
    {"reverse", onReverse},
    {"forward", onForward},
    {"stop", onStop},
    {"open", onOpen},
    {"close", onClose},
    {"step", onStepForward},
    {"stepback", onStepBackward},
    {"escalatorstart", escalatorStart},
    {"escalatorstop", escalatorStop}
};
