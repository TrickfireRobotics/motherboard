#include "usbTaskHelper.hpp"

CommandType getCommandTypeRaw(char* data, int arraySize){

    // Max length is 7 because "STEPPER" is seven
    // but we will fill until the first WHITESPACE character
    char firstWord[7];

    int index = 0;

    // while we are not at the end of the array nor have we seen a white space, keep going
    while (index < arraySize && data[index] != ' ') {
        firstWord[index] = data[index];
        index++;
    }

    printf("firstWord[0] %c\n", firstWord[0]);
    printf("firstWord[1] %c\n", firstWord[1]);
    printf("firstWord[2] %c\n", firstWord[2]);
    printf("firstWord[3] %c\n", firstWord[3]);
    printf("firstWord[4] %c\n", firstWord[4]);
    printf("firstWord[5] %c\n", firstWord[5]);
    printf("firstWord[6] %c\n", firstWord[6]);

    // 5 bc POWER
    char secondWord[5];
    int secondWordIndex = 0;
    index++;
    while (index < arraySize && data[index] != ' ') {
        secondWord[secondWordIndex] = data[index];
        index++;
        secondWordIndex++;
    }

    printf("secondWord[0] %c\n", secondWord[0]);
    printf("secondWord[1] %c\n", secondWord[1]);
    printf("secondWord[2] %c\n", secondWord[2]);
    printf("secondWord[3] %c\n", secondWord[3]);
    printf("secondWord[4] %c\n", secondWord[4]);

    if (firstWord[0] == 's' &&
        firstWord[1] == 't' &&
        firstWord[2] == 'e' &&
        firstWord[3] == 'p' &&
        firstWord[4] == 'p' &&
        firstWord[5] == 'e' &&
        firstWord[6] == 'r') {

        if (secondWord[0] == 'g' &&
            secondWord[1] == 'e' &&
            secondWord[2] == 'n') {
            return CommandType::STEPPER_GEN;
        }

        if (secondWord[0] == 'c' && 
            secondWord[1] == 'o' &&
            secondWord[2] == 'n' &&
            secondWord[3] == 'f') {
            return CommandType::STEPPER_CONF;
        }

        if (secondWord[0] == 'p' &&
            secondWord[1] == 'o' &&
            secondWord[2] == 'w' &&
            secondWord[3] == 'e' &&
            secondWord[4] == 'r') {
            return CommandType::STEPPER_POWER;
        }
    }

    return CommandType::UNKNOWN;
}

void updateStepperGeneral(char* data, int arraySize){
    printf("STEPPER GEN\n");

}

void updateStepperConfig(char* data, int arraySize){
    printf("STEPPER CONF\n");
}

void updateStepperPower(char* data, int arraySize){
    printf("STEPPER POWER\n");
}


void updateStepperPWM(char* data, int arraySize){
    printf("PWM\n");
}

void updateStepperLIGHT(char* data, int arraySize){
    printf("LIGHT\n");

}