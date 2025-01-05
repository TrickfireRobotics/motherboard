#ifndef USB_TASK_HELPER_H
#define USB_TASK_HELPER_H

#include "main.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <pico/stdio_usb.h>

enum CommandType{
    STEPPER_GEN = 0,
    STEPPER_CONF = 1,
    STEPPER_POWER = 2,
    PWM = 3,
    LIGHT = 4,
    UNKNOWN = -1
};


// ===== Function prototypes =====
CommandType getCommandTypeRaw(char* data, int arraySize);

void updateStepperGeneral(char* data, int arraySize);
void updateStepperConfig(char* data, int arraySize);
void updateStepperPower(char* data, int arraySize);
void updateStepperPWM(char* data, int arraySize);
void updateStepperLIGHT(char* data, int arraySize);

bool isStringEqual(char* subject, int subjectLength, char* target, int targetLength);


#endif