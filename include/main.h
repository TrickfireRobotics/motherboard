#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "usbTaskHelper.hpp"
#include <tusb.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <pico/stdio_usb.h>

// ===== I2C =====
#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3

// ===== Expander address =====
#define EXPANDER1_ADDR 0x20
#define EXPANDER2_ADDR 0x21
#define EXPANDER3_ADDR 0x22
#define EXPANDER4_ADDR 0x23

// ===== Expander registers =====
#define TCA9555_INPUT_PORT_0 0x00
#define TCA9555_INPUT_PORT_1 0x01
#define TCA9555_OUTPUT_PORT_0 0x02
#define TCA9555_OUTPUT_PORT_1 0x03
#define TCA9555_POLARITY_PORT_0 0x04
#define TCA9555_POLARITY_PORT_1 0x05
#define TCA9555_CONFIG_PORT_0 0x06
#define TCA9555_CONFIG_PORT_1 0x07

// ===== Stepper Motors, Servos, and LED ports (not pins)
#define NUM_STEPPERS 6
#define NUM_SERVOS 6

#define STEPPER_1_DIR 17
#define STEPPER_1_STEP 16
#define STEPPER_1_SLEEP 15
#define STEPPER_1_RST 14
#define STEPPER_1_EN 10
#define STEPPER_1_MS1 11
#define STEPPER_1_MS2 12
#define STEPPER_1_MS3 13

#define STEPPER_2_DIR 7
#define STEPPER_2_STEP 6
#define STEPPER_2_SLEEP 5
#define STEPPER_2_RST 4
#define STEPPER_2_EN 0
#define STEPPER_2_MS1 1
#define STEPPER_2_MS2 2
#define STEPPER_2_MS3 3

#define STEPPER_3_DIR 17
#define STEPPER_3_STEP 16
#define STEPPER_3_SLEEP 15
#define STEPPER_3_RST 14
#define STEPPER_3_EN 10
#define STEPPER_3_MS1 11
#define STEPPER_3_MS2 12
#define STEPPER_3_MS3 13

#define STEPPER_4_DIR 7
#define STEPPER_4_STEP 6
#define STEPPER_4_SLEEP 5
#define STEPPER_4_RST 4
#define STEPPER_4_EN 0
#define STEPPER_4_MS1 1
#define STEPPER_4_MS2 2
#define STEPPER_4_MS3 3

#define STEPPER_5_DIR 17
#define STEPPER_5_STEP 16
#define STEPPER_5_SLEEP 15
#define STEPPER_5_RST 14
#define STEPPER_5_EN 10
#define STEPPER_5_MS1 11
#define STEPPER_5_MS2 12
#define STEPPER_5_MS3 13

#define STEPPER_6_DIR 7
#define STEPPER_6_STEP 6
#define STEPPER_6_SLEEP 5
#define STEPPER_6_RST 4
#define STEPPER_6_EN 0
#define STEPPER_6_MS1 1
#define STEPPER_6_MS2 2
#define STEPPER_6_MS3 3

#define PWM1 7
#define PWM2 6
#define PWM3 5
#define PWM4 4
#define PWM5 3
#define PWM6 2

#define LED_RED 17
#define LED_BLUE 16
#define LED_GREEN 15

// === usbTask specific values ===
#define MAX_USB_INPUT_BUFFER_CHARS 96

// ===== group into const for easy access =====
const uint8_t stepperDirPins[NUM_STEPPERS] = {
    STEPPER_1_DIR,
    STEPPER_2_DIR,
    STEPPER_3_DIR,
    STEPPER_4_DIR,
    STEPPER_5_DIR,
    STEPPER_6_DIR};

const uint8_t stepperStepPins[NUM_STEPPERS] = {
    STEPPER_1_STEP,
    STEPPER_2_STEP,
    STEPPER_3_STEP,
    STEPPER_4_STEP,
    STEPPER_5_STEP,
    STEPPER_6_STEP};

const uint8_t stepperSleepPins[NUM_STEPPERS] = {
    STEPPER_1_SLEEP,
    STEPPER_2_SLEEP,
    STEPPER_3_SLEEP,
    STEPPER_4_SLEEP,
    STEPPER_5_SLEEP,
    STEPPER_6_SLEEP};

const uint8_t stepperRstPins[NUM_STEPPERS] = {
    STEPPER_1_RST,
    STEPPER_2_RST,
    STEPPER_3_RST,
    STEPPER_4_RST,
    STEPPER_5_RST,
    STEPPER_6_RST};

const uint8_t stepperEnPins[NUM_STEPPERS] = {
    STEPPER_1_EN,
    STEPPER_2_EN,
    STEPPER_3_EN,
    STEPPER_4_EN,
    STEPPER_5_EN,
    STEPPER_6_EN};

const uint8_t stepperMs1Pins[NUM_STEPPERS] = {
    STEPPER_1_MS1,
    STEPPER_2_MS1,
    STEPPER_3_MS1,
    STEPPER_4_MS1,
    STEPPER_5_MS1,
    STEPPER_6_MS1};

const uint8_t stepperMs2Pins[NUM_STEPPERS] = {
    STEPPER_1_MS2,
    STEPPER_2_MS2,
    STEPPER_3_MS2,
    STEPPER_4_MS2,
    STEPPER_5_MS2,
    STEPPER_6_MS2};

const uint8_t stepperMs3Pins[NUM_STEPPERS] = {
    STEPPER_1_MS3,
    STEPPER_2_MS3,
    STEPPER_3_MS3,
    STEPPER_4_MS3,
    STEPPER_5_MS3,
    STEPPER_6_MS3};

const uint8_t pwmPins[NUM_SERVOS] = {
    PWM1,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6};

// ===== Task priorities =====
// smaller number = lower priority
// idle task = 0 priority
#define USB_TASK_PRIORITY 2
#define LIGHT_TASK_PRIORITY 1
#define STEPPER_TASK_PRIORITY 1
#define PWM_SERVO_TASK_PRIOTITY 1

// ===== structs for the stepper motor and servo =====
typedef struct
{
    bool isEnable;
    bool isSleep;
    bool isReset;
    bool isEnableDirty;
    bool isSleepDirty;
    bool isResetDirty;
    bool isMsDirty;
    bool isDirDirty;
    bool ignoreTargetPos; // true = we spin continuously | false = we spin until we reach the targetPosition
    uint8_t expanderAddr;
    uint8_t MS1;
    uint8_t MS2;
    uint8_t MS3;
    uint8_t dir;              // 1 = positive command = clockwise | 0 = negative command = counter clockwise
    uint16_t currentPosition; // in terms of stepper steps
    uint16_t targetPosition;  // in terms of stepper steps | Issue(?): what if we want the position to go negative?
    uint32_t stepInterval;    // in us
    uint32_t lastStepTime;    // in us
} StepperMotor;

// time unit = ms? us?
typedef struct
{
    // uint8_t port;
    bool isOn;
    uint32_t startTime;
    uint32_t onTime;  // in us
    uint32_t offTime; // in us
} Servo;

// External variables defined in main.cpp
extern Servo servos[NUM_SERVOS];
extern StepperMotor stepperMotors[NUM_STEPPERS];

extern SemaphoreHandle_t stepperMutexes[NUM_STEPPERS];
extern SemaphoreHandle_t servoMutexes[NUM_SERVOS];
extern SemaphoreHandle_t writeToUsbMutex;

// ===== Function prototypes =====
void usbTask(void *params);
void lightTask(void *params);
void stepperMotorTask(void *params);
void pwmServoTask(void *params);

void i2cSetPin(uint8_t expanderAddress, uint8_t pinNumber, bool value);

#endif /* MAIN_H */