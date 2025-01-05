#include "usbTaskHelper.hpp"

CommandType getCommandTypeRaw(char* data, int arraySize){

    char stringStepper[7] = {'s','t','e','p','p','e','r'};
    char stringGen[3] = {'g','e','n'};
    char stringConf[4] = {'c','o','n','f'};
    char stringPower[5] = {'p','o','w','e','r'};
    char stringPwm[3] = {'p','w','m'};
    char stringLight[5] = {'l','i','g','h','t'};


    // Max length is 7 because "STEPPER" is seven
    // but we will fill until the first WHITESPACE character
    char firstWord[7];
    int index = 0;

    // while we are not at the end of the array nor have we seen a white space, keep going
    while (index < arraySize && data[index] != ' ') {
        firstWord[index] = data[index];
        index++;
    }


    // 5 b/c POWER
    char secondWord[5];
    int secondWordIndex = 0;
    index++;
    while (index < arraySize && data[index] != ' ') {
        secondWord[secondWordIndex] = data[index];
        index++;
        secondWordIndex++;
    }

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
    else if (isStringEqual(firstWord, 7, stringPwm, 3)) {
        return CommandType::PWM;
    }
    else if (isStringEqual(firstWord, 7, stringLight, 5)) {
        return CommandType::LIGHT;
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

    int port = data[12] - '0';
    float velocity = 0;
    float position = 0;

    // --- Converting the VELOCITY into its IEEE 754 floating point number ---

    // Step 1: Using an unsigned int (to avoid 2's complement) we fill the 32 bits from
    // the message sent
    unsigned int rawVelocity = 0;
    int bitwiseOffset = 31;

    for (int dataIndex = 14; dataIndex < 46; dataIndex++) {
        int bit = data[dataIndex] - '0';
        rawVelocity = rawVelocity | (bit << bitwiseOffset);
        bitwiseOffset--;
    }

    // Step 2: Do some "Type Punning"

    // Gremlin code to convert uint to a float - not type casting, but physically changing how the
    // the mcu reads the bits in memory by manipulating some pointer types
    unsigned int* rawVelocityPTR = &rawVelocity; // get pointer of the uint
    float* floatPTR = (float*)rawVelocityPTR; // convert that pointer into a float
    velocity = *floatPTR; // convert that pointer into a pure ieee 754 single floating number


    printf("The port number is %u\n", port);
    printf("The integer number is %u\n", rawVelocity);
    printf("The floating number is %f\n", velocity);




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

float readAndConvertRawFloatBits(char* data, int arraySize, int start, int end){
    
}