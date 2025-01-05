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
    float velocity = readAndConvertRawFloatBits(data, arraySize, 14, 46);
    float position = readAndConvertRawFloatBits(data, arraySize, 47, 79);

    
}


void updateStepperConfig(char* data, int arraySize){
    printf("STEPPER CONF\n");

    int port = data[13] - '0';
    int ms1 = data[15] - '0';
    int ms2 = data[17] - '0';
    int ms3 = data[19] - '0';

    printf("PORT %d\n", port);
    printf("MS1 %d\n", ms1);
    printf("MS2 %d\n", ms2);
    printf("MS3 %d\n", ms3);
}

void updateStepperPower(char* data, int arraySize){
    printf("STEPPER POWER\n");

    int port = data[14] - '0';
    int enable = data[16] - '0'; // integer 0 is 0; integer 1 is 1; integer 59 is k
    int sleep = data[18] - '0';
    int reset = data[20] - '0';

    printf("PORT %d\n", port);
    printf("enable %d\n", enable);
    printf("sleep %d\n", sleep);
    printf("reset %d\n", reset);


}


void updateStepperPWM(char* data, int arraySize){
    printf("PWM\n");

    int port = data[4] - '0';
    float dutyCycle = readAndConvertRawFloatBits(data, arraySize, 6, 38);
    float frequency = readAndConvertRawFloatBits(data, arraySize, 39, 71);

    printf("PORT %d\n", port);
    printf("Duty Cycle %f\n", dutyCycle);
    printf("Frequency %f\n", frequency);

}

void updateStepperLIGHT(char* data, int arraySize){
    printf("LIGHT\n");

    char lightType = data[6];
    int state = data[8] - '0';

    printf("lightType %c\n", lightType);
    printf("state%d\n", state);

}

float readAndConvertRawFloatBits(char* data, int arraySize, int start, int end){

    float finalNumber = 0;

    // --- Converting the VELOCITY into its IEEE 754 floating point number ---

    // Step 1: Using an unsigned int (to avoid 2's complement) we fill the 32 bits from
    // the message sent
    unsigned int rawFinalNumber = 0;
    int bitwiseOffset = 31;

    for (int dataIndex = start; dataIndex < end; dataIndex++) {
        int bit = data[dataIndex] - '0';
        rawFinalNumber = rawFinalNumber | (bit << bitwiseOffset);
        bitwiseOffset--;
    }

    // Step 2: Do some "Type Punning"

    // Gremlin code to convert uint to a float - not type casting, but physically changing how the
    // the mcu reads the bits in memory by manipulating some pointer types
    unsigned int* rawVelocityPTR = &rawFinalNumber; // get pointer of the uint
    float* floatPTR = (float*)rawVelocityPTR; // convert that pointer into a float
    finalNumber = *floatPTR; // convert that pointer into a pure ieee 754 single floating number

    return finalNumber;


}