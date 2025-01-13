#ifndef USB_TASK_HELPER_H
#define USB_TASK_HELPER_H

#include <stdio.h>
#include <stdlib.h>

#include "main.h"

// === Pi Pico Includes ===
#include "pico/stdlib.h"
#include <pico/stdio_usb.h>

// === FreeRTOS Includes ===
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

enum CommandType
{
    STEPPER_GEN = 0,
    STEPPER_CONF = 1,
    STEPPER_POWER = 2,
    PWM = 3,
    LIGHT = 4,
    GET_MOTHERBOARD_DEVICE = 5,
    GET_MOTHERBOARD_DEBUG_DEVICE = 6,
    UNKNOWN = -1
};

enum StepResolution
{
    FULL = 1,
    HALF = 2,
    QUARTER = 4,
    EIGHTH = 8,
    SIXTEENTH = 16
};

// ===== Function prototypes =====
CommandType getCommandTypeRaw(char *data, int arraySize);

void updateStepperGeneral(char *data, int arraySize);
void updateStepperConfig(char *data, int arraySize);
void updateStepperPower(char *data, int arraySize);
void updateStepperPWM(char *data, int arraySize);
void updateStepperLIGHT(char *data, int arraySize);
void sendMBDeviceData(char *data, int arraySize);
void sendMBDebugDeviceData(char *data, int arraySize);

bool isStringEqual(char *subject, int subjectLength, char *target, int targetLength);
float readAndConvertRawBitsIntoFloat(char *data, int arraySize, int start, int end);
StepResolution getStepperMotorStepRes(uint8_t port);

#endif