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
        printf("hello from stepperMotorTask\n");
        sleep_ms(1000);
    }
    // TODO: add definition
}

// servo task
void pwmServoTask(void *params)
{
    while (true)
    {
        printf("hello from pwmServoTask\n");
        sleep_ms(1000);
    }
    // TODO: add definition
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