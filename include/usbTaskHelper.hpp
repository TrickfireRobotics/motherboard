#ifndef USB_TASK_HELPER_H
#define USB_TASK_HELPER_H

enum CommandType{
    STEPPER_GEN = 0,
    STEPPER_CONF = 1,
    STEPPER_POWER = 2,
    PWM = 3,
    LIGHT = 4
};



CommandType getCommandTypeRaw(char* data, int arraySize);


#endif