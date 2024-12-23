#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// ===== Expander address =====
#define EXPANDER1_ADDR 0x20
#define EXPANDER2_ADDR 0x21
#define EXPANDER3_ADDR 0x22
#define EXPANDER4_ADDR 0x23

// ===== Stepper Motors, Servos, and LED pins
#define NUM_STEPPERS            6
#define NUM_SERVOS              6

#define STEPPER_1_DIR           20
#define STEPPER_1_STEP          19
#define STEPPER_1_SLEEP         18
#define STEPPER_1_RST           17
#define STEPPER_1_EN            13
#define STEPPER_1_MS1           14
#define STEPPER_1_MS2           15
#define STEPPER_1_MS3           16

#define STEPPER_2_DIR           11
#define STEPPER_2_STEP          10
#define STEPPER_2_SLEEP         9
#define STEPPER_2_RST           8
#define STEPPER_2_EN            4
#define STEPPER_2_MS1           5
#define STEPPER_2_MS2           6
#define STEPPER_2_MS3           7

#define STEPPER_3_DIR           20
#define STEPPER_3_STEP          19
#define STEPPER_3_SLEEP         18
#define STEPPER_3_RST           17
#define STEPPER_3_EN            13
#define STEPPER_3_MS1           14
#define STEPPER_3_MS2           15
#define STEPPER_3_MS3           16

#define STEPPER_4_DIR           11
#define STEPPER_4_STEP          10
#define STEPPER_4_SLEEP         9
#define STEPPER_4_RST           8
#define STEPPER_4_EN            4
#define STEPPER_4_MS1           5
#define STEPPER_4_MS2           6
#define STEPPER_4_MS3           7

#define STEPPER_5_DIR           20
#define STEPPER_5_STEP          19
#define STEPPER_5_SLEEP         18
#define STEPPER_5_RST           17
#define STEPPER_5_EN            13
#define STEPPER_5_MS1           14
#define STEPPER_5_MS2           15
#define STEPPER_5_MS3           16

#define STEPPER_6_DIR           11
#define STEPPER_6_STEP          10
#define STEPPER_6_SLEEP         9
#define STEPPER_6_RST           8
#define STEPPER_6_EN            4
#define STEPPER_6_MS1           5
#define STEPPER_6_MS2           6
#define STEPPER_6_MS3           7

#define PWM1                    11
#define PWM2                    10
#define PWM3                    9
#define PWM4                    8
#define PWM5                    7
#define PWM6                    6

#define LED_RED                 20
#define LED_BLUE                19
#define LED_GREEN               18

// === usbTask specific values ===
#define MAX_USB_INPUT_BUFFER_CHARS 96

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


// ===== Declare global data =====
// extern StepperMotor stepper0;
// extern StepperMotor stepper1;
// extern StepperMotor stepper2;
// extern StepperMotor stepper3;
// extern StepperMotor stepper4;
// extern StepperMotor stepper5;

// extern Servo servo0;
// extern Servo servo1;
// extern Servo servo2;
// extern Servo servo3;
// extern Servo servo4;
// extern Servo servo5;

// ===== Function prototypes =====
void usbTask(void *params);
void lightTask(void *params);
void stepperMotorTask(void *params);
void pwmServoTask(void *params);

void i2cSetPin(uint8_t expanderAddress, uint8_t pinNumber, bool value);

#endif /* MAIN_H */