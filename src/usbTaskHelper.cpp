#include "usbTaskHelper.hpp"

CommandType getCommandTypeRaw(char* data, int arraySize){

    char stringStepper[7] = {'s','t','e','p','p','e','r'};
    char stringGen[3] = {'g','e','n'};
    char stringConf[4] = {'c','o','n','f'};
    char stringPower[5] = {'p','o','w','e','r'};

    // Max length is 7 because "STEPPER" is seven
    // but we will fill until the first WHITESPACE character
    char firstWord[7];
    int index = 0;

    // while we are not at the end of the array nor have we seen a white space, keep going
    while (index < arraySize && data[index] != ' ') {
        firstWord[index] = data[index];
        index++;
    }

    // printf("firstWord[0] %c\n", firstWord[0]);
    // printf("firstWord[1] %c\n", firstWord[1]);
    // printf("firstWord[2] %c\n", firstWord[2]);
    // printf("firstWord[3] %c\n", firstWord[3]);
    // printf("firstWord[4] %c\n", firstWord[4]);
    // printf("firstWord[5] %c\n", firstWord[5]);
    // printf("firstWord[6] %c\n", firstWord[6]);

    // 5 bc POWER
    char secondWord[5];
    int secondWordIndex = 0;
    index++;
    while (index < arraySize && data[index] != ' ') {
        secondWord[secondWordIndex] = data[index];
        index++;
        secondWordIndex++;
    }

    // printf("secondWord[0] %c\n", secondWord[0]);
    // printf("secondWord[1] %c\n", secondWord[1]);
    // printf("secondWord[2] %c\n", secondWord[2]);
    // printf("secondWord[3] %c\n", secondWord[3]);
    // printf("secondWord[4] %c\n", secondWord[4]);

    if (isStringEqual(firstWord, 7, stringStepper, 7)) {
        if (isStringEqual(secondWord, 5, stringGen, 3)) {
            return CommandType::STEPPER_GEN;
        }
        else if (isStringEqual(secondWord, 5, stringConf, 3)) {
            return CommandType::STEPPER_CONF;
        }
        else if(isStringEqual(secondWord, 5, stringPower, 5)){
            return CommandType::STEPPER_POWER;
        }
    }

    return CommandType::UNKNOWN;
}

bool isStringEqual(char* subject, int subjectLength, char* target, int targetLength){
    if (subjectLength < targetLength) {
        return false;
    }

    for (int index = 0; index < targetLength; index++) {
        if (subject[index] != target[index]) {
            return false;
        }
    }

    return true;
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