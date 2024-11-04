#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <pico/stdio_usb.h>
//#include "ff_stdio.h"


//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"



TaskHandle_t usbInTaskHandle;
TaskHandle_t usbOutTaskHandle;

QueueHandle_t dataToHostQueue;

void usbDataIn(void *param)
{
    while (!stdio_usb_connected())
    {
        sleep_ms(100);
    }
    printf("stdio_usb_connected()\n");

    while (true) {
        printf("hello from task\n");
        sleep_ms(1000);
    }
    
}


void tenSecDebugLED(){
    // Blink 5 times in 10 seconds
    for (int index = 0; index < 10; index++) {
        gpio_put(25,index % 2);
        sleep_ms(1000);
        printf("%d \n", index);
    }
}

void onBootInit(){
    stdio_init_all();

    // This is the LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
}

int main(int argc, char** argv) {
    onBootInit();

    #if (DEBUG)
        tenSecDebugLED();
    #endif

    xTaskCreate(usbDataIn, "USB_DATA_IN", configMINIMAL_STACK_SIZE * 2, NULL, 1, &usbInTaskHandle);

    vTaskStartScheduler();

    return 0;
}