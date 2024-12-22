#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <pico/stdio_usb.h>
#include "main.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// mutex for each motors and servos
SemaphoreHandle_t stepperMutexes[NUM_STEPPERS];
SemaphoreHandle_t servoMutexes[NUM_SERVOS];

// handle for each task
TaskHandle_t usbInTaskHandle;
TaskHandle_t usbOutTaskHandle;

TaskHandle_t usbTaskHandle;
TaskHandle_t lightTaskHandle;
TaskHandle_t stepperMotorTaskHandle;
TaskHandle_t pwmServoTaskHandle;

QueueHandle_t dataToHostQueue;

// declare stepper motors and servos
StepperMotor stepperMotors[NUM_STEPPERS];
Servo servos[NUM_SERVOS];

// example task
void exampleTask(void *param)
{
    while (!stdio_usb_connected())
    {
        sleep_ms(100);
    }
    printf("stdio_usb_connected()\n");

    while (true)
    {
        printf("hello from example task\n");
        sleep_ms(1000);
    }

    // debug for checking stack size
    // then set the stack size to the appropriate size:
    // see: https://www.freertos.org/Documentation/02-Kernel/04-API-references/03-Task-utilities/04-uxTaskGetStackHighWaterMark
    printf("High water mark (words): %lu\n", uxTaskGetStackHighWaterMark(NULL));
}

// usb task
void usbTask(void *params)
{
    while (true)
    {
        printf("hello from usbTask\n");
        sleep_ms(1000);
    }
    // TODO: add definition
}

// light task
void lightTask(void *params)
{
    while (true)
    {
        printf("hello from lightTask\n");
        sleep_ms(1000);
    }
    // TODO: add definition
}

// stepper motor task
void stepperMotorTask(void *params)
{
    while (true)
    {
        // go through each stepper motor
        for (uint8_t i = 0; i < NUM_STEPPERS; ++i)
        {
            // attempt to obtain the mutex
            if (stepperMutexes[i] != NULL)
            {
                // wait for 10 tick, if can't move on to next servo
                if (xSemaphoreTake(stepperMutexes[i], (TickType_t)10) == pdTRUE)
                {
                    // check for enable, sleep, and reset first
                    if (stepperMotors[i].isEnableDirty)
                    {
                        // reset the dirty bit
                        stepperMotors[i].isEnableDirty = false;

                        // EN active low
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperEnPins[i], !stepperMotors[i].isEnable);
                    }
                    else if (stepperMotors[i].isSleepDirty)
                    {
                        // reset the dirty bit
                        stepperMotors[i].isSleepDirty = false;

                        // SLP active low
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperSleepPins[i], !stepperMotors[i].isSleep);
                    }
                    else if (stepperMotors[i].isResetDirty)
                    {
                        // reset the dirty bit
                        stepperMotors[i].isResetDirty = false;

                        // TODO: pull low into reset state, unless pull high again
                        // double check to see if the cmd to turn high comes from
                        // the uart or do we just pull it low for a certain time
                        // and pull it back high
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperRstPins[i], !stepperMotors[i].isReset);

                        // reset the position var as well
                        if (stepperMotors[i].isReset)
                        {
                            stepperMotors[i].currentPosition = 0;
                            stepperMotors[i].targetPosition = 0;
                        }
                    }

                    // if disable, sleep, and reset, return the mutex and continue the loop
                    if (!stepperMotors[i].isEnable || stepperMotors[i].isSleep || stepperMotors[i].isReset)
                    {
                        xSemaphoreGive(stepperMutexes[i]);
                        continue;
                    }

                    // for other cases:
                    // check if micro-step resolution has changed
                    if (stepperMotors[i].isMsDirty)
                    {
                        // reset the dirty bit
                        stepperMotors[i].isMsDirty = false;

                        // send cmd to ms pins
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperMs1Pins[i], stepperMotors[i].MS1);
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperMs2Pins[i], stepperMotors[i].MS2);
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperMs3Pins[i], stepperMotors[i].MS3);
                    }

                    // check if direction has changed
                    if (stepperMotors[i].isDirDirty)
                    {
                        // reset the firty bit
                        stepperMotors[i].isDirDirty = false;

                        // send cmd to change direction
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperDirPins[i], stepperMotors[i].dir);
                    }

                    /*
                        If current pos < target pos
                        check step time & perform the step
                        else continue the loop
                    */
                    if (stepperMotors[i].currentPosition < stepperMotors[i].targetPosition &&
                        ((xTaskGetTickCount() - stepperMotors[i].lastStepTime) >= stepperMotors[i].stepInterval))
                    {
                        // increase the step count
                        ++stepperMotors[i].currentPosition;

                        // return the mutex
                        xSemaphoreGive(stepperMutexes[i]);

                        // send step cmd
                        i2cSetPin(stepperMotors[i].expanderAddr, stepperStepPins[i], 1);

                        // minimum pulse width of A4988 is 1 us for both high and low
                        sleep_us(1);

                        i2cSetPin(stepperMotors[i].expanderAddr, stepperStepPins[i], 0);
                    }
                    // for other cases, remember to return the mutexes as well
                    else
                    {
                        xSemaphoreGive(stepperMutexes[i]);
                    }
                }
            }
        }
    }
}
}

// servo task
void pwmServoTask(void *params)
{
    /*
        For each servo:
        Attempt to take the semaphore
            - not succeed -> block wait for 10 ms -> move on
            - succeed -> check time, send cmd, release mutex
    */
    while (true)
    {
        // go through each servo
        for (uint8_t i = 0; i < NUM_SERVOS; ++i)
        {
            if (servoMutexes[i] != NULL)
            {
                // wait for 10 tick, if can't move on to next servo
                if (xSemaphoreTake(servoMutexes[i], (TickType_t)10) == pdTRUE)
                {
                    // if servo is on, checks against the on period
                    if (servos[i].isOn && ((xTaskGetTickCount() - servos[i].startTime) >= servos[i].onTime))
                    {
                        // turn it off
                        i2cSetPin(EXPANDER4_ADDR, pwmPins[i], 0);

                        // reset the start time
                        servos[i].startTime = xTaskGetTickCount();
                        servos.isOn = false;
                    }
                    // else if it's off, check against the off period
                    else if (!servos[i].isOn && ((xTaskGetTickCount() - servos[i].startTime) >= servos[i].offTime))
                    {
                        // turn it on
                        i2cSetPin(EXPANDER4_ADDR, pwmPins[i], 1);

                        // reset the start time
                        servos[i].startTime = xTaskGetTickCount();
                        servos.isOn = true;
                    }

                    // return the mutex
                    xSemaphoreGive(servoMutexes[i]);
                }
            }
        }
    }
}

// for sending i2c cmd
void i2cSetPin(uint8_t expanderAddress, uint8_t pinNumber, bool value)
{
    // TODO: add definition
}

// example debug function
void tenSecDebugLED()
{
    // Blink 5 times in 10 seconds
    for (int index = 0; index < 10; index++)
    {
        gpio_put(25, index % 2);
        sleep_ms(1000);
        printf("%d \n", index);
    }
}

void onBootInit()
{
    stdio_init_all();

    // This is the LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
}

int main(int argc, char **argv)
{
    onBootInit();

#if (DEBUG)
    tenSecDebugLED();
#endif

    // creates mutexes
    // these uses RAM space (dynamic mem). If constraint is needed, use static mutex.
    // usage example, see: https://www.freertos.org/Documentation/02-Kernel/04-API-references/10-Semaphore-and-Mutexes/06-xSemaphoreCreateMutex
    for (uint8_t i = 0; i < NUM_STEPPERS; ++i)
    {
        stepperMutexes[i] = xSemaphoreCreateMutex();

        // check if success
        if (stepperMutexes[i] == NULL)
            printf("Failed to create mutex for stepper %d\n", i + 1);
    }

    for (uint8_t i = 0; i < NUM_SERVOS; ++i)
    {
        servoMutexes[i] = xSemaphoreCreateMutex();

        // check if success
        if (servoMutexes[i] == NULL)
            printf("Failed to create mutex for servo %d\n", i + 1);
    }

    // example task
    // note that the stack size is in words, NOT bytes
    // (see doc for explanation)
    xTaskCreate(exampleTask, "USB_DATA_IN", configMINIMAL_STACK_SIZE * 2, NULL, 1, &usbInTaskHandle);

    // all default stack size, change if needed
    // min stack size is 128 words
    xTaskCreate(usbTask,                  // task func name
                "USB_TASK",               // name in char, max 16 chars
                configMINIMAL_STACK_SIZE, // stack depth
                NULL,                     // params
                USB_TASK_PRIORITY,        // priority
                &usbOutTaskHandle);       // handle

    xTaskCreate(lightTask,
                "LIGHT_TASK",
                configMINIMAL_STACK_SIZE,
                NULL,
                LIGHT_TASK_PRIORITY,
                &lightTaskHandle);

    xTaskCreate(stepperMotorTask,
                "STEPPER_TASK",
                configMINIMAL_STACK_SIZE,
                NULL,
                STEPPER_TASK_PRIORITY,
                &stepperMotorTaskHandle);

    xTaskCreate(pwmServoTask,
                "SERVO_TASK",
                configMINIMAL_STACK_SIZE,
                NULL,
                PWM_SERVO_TASK_PRIOTITY,
                &pwmServoTaskHandle);

    vTaskStartScheduler();

    return 0;
}