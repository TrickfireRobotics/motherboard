#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// ===== Expander address =====
#define EXPANDER1_ADDR 0x20
#define EXPANDER2_ADDR 0x21
#define EXPANDER3_ADDR 0x22
#define EXPANDER4_ADDR 0x23

// ===== Task priorities =====
// smaller number = lower priority
// idle task = 0 priority
#define USB_TASK_PRIORITY       2
#define LIGHT_TASK_PRIORITY     1
#define STEPPER_TASK_PRIORITY   1
#define PWM_SERVO_TASK_PRIOTITY 1

// ===== structs for the stepper motor and servo =====
typedef struct {
    bool isEnable;
    bool isSleep;
    bool isReset;
    bool isEnableDirty;
    bool isSleepDirty;
    bool isResetDirty;
    uint8_t port;
    uint8_t MS1;
    uint8_t MS2;
    uint8_t MS3;
    uint16_t startTime;
    float velocity;
    float position;
} StepperMotor;

typedef struct {
    uint16_t startTime;
    uint16_t onTime;
    uint8_t port;
} Servo;


// ===== Function prototypes =====
void usbTask(void *params);
void lightTask(void *params);
void stepperMotorTask(void *params);
void pwmServoTask(void *params);

void i2cSetPin(uint8_t expanderAddress, uint8_t pinNumber);

#endif /* MAIN_H */