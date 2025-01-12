#include "usbTaskHelper.hpp"

/**
 * Based on the string input, determine what command is sent.
 * Returns an entry from CommandType
 * 
 * 
 */
CommandType getCommandTypeRaw(char* data, int arraySize){

    char stringStepper[7] = {'s','t','e','p','p','e','r'};
    char stringGen[3] = {'g','e','n'};
    char stringConf[4] = {'c','o','n','f'};
    char stringPower[5] = {'p','o','w','e','r'};
    char stringPwm[3] = {'p','w','m'};
    char stringLight[5] = {'l','i','g','h','t'};
    char stringGetMBDevice[6] = {'g','e','t','m','b', 'd'};
    char stringGetMBDebugDevice[6] = {'g','e','t','m','b', 'b'};


    // Max length is 7 because "STEPPER" is seven
    // but we will fill until the first WHITESPACE character
    char firstWord[7];
    int index = 0;

    // While we are not at the end of the array nor have we seen a white space, keep going
    while (index < arraySize && data[index] != ' ') {
        firstWord[index] = data[index];
        index++;
    }


    // Length of 5 because keyword "power"
    char secondWord[5];
    int secondWordIndex = 0;
    index++;
    while (index < arraySize && data[index] != ' ') {
        secondWord[secondWordIndex] = data[index];
        index++;
        secondWordIndex++;
    }

    // Actual comparisons
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
    else if (isStringEqual(firstWord, 7, stringGetMBDevice, 6)){
        return CommandType::GET_MOTHERBOARD_DEVICE;
    }
    else if (isStringEqual(firstWord, 7, stringGetMBDebugDevice, 6)) {
        return CommandType::GET_MOTHERBOARD_DEBUG_DEVICE;
    }

    return CommandType::UNKNOWN;
}

/**
 * Helper function to see if two char arrays are equal in characters
 * 
 * returns: true if equal, false if not
 */
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


/**
 * Handles the "stepper gen..." command. 
 * Look at the Notion docs for more info about the command definition:
 * https://www.notion.so/trickfire/Software-USB-Communication-Commands-1651fd41ff5b80adbc37eacd546f46b2
 */
void updateStepperGeneral(char* data, int arraySize){
    bool isPosNaN = false;

    int port = data[12] - '0'; // "- '0'" because that maps to real ints from char
    float velocity = readAndConvertRawBitsIntoFloat(data, arraySize, 14, 46);
    float position = 0.0;

    if (data[47] == 'n' && data[48] == 'a' && data[47] == 'n') {
        isPosNaN = true;
    }
    else {
        //TODO: better error checking. What if we sent "00101aAgb"? The code would die.
        position = readAndConvertRawBitsIntoFloat(data, arraySize, 47, 79);
    }

    // Now update the motherboard logical state

    // Lock the stepper semaphore
    SemaphoreHandle_t* semaphore = &stepperMutexes[port];
    
    if (xSemaphoreTake(*semaphore, 10)) {
        
        StepperMotor* stepper = &stepperMotors[port];

        if (velocity < 0.0) { // go ccw
            stepper->dir = 0;
        }
        else { // go cw
            stepper->dir = 1;
        }
        stepper->isDirDirty = true;

        // NEMA 17-size motors have 200steps/rev
        // This means 1 STEP = 1.8deg = 0.005rev = 0.031415rad
        // TODO: make this a config setting
        StepResolution stepResolution = getStepperMotorStepRes(port);
        int totalStepsPerRev = 200 * stepResolution;

        if (!isPosNaN) {
            stepper->targetPosition = totalStepsPerRev * position;
        }

        stepper->ignoreTargetPos = isPosNaN;

        stepper->stepInterval = 1000000.0 / (totalStepsPerRev * velocity);

        xSemaphoreGive(*semaphore);
    }
}


/**
 * Handles the "stepper conf..." command
 * Look at the Notion docs for more info about the command definition:
 * https://www.notion.so/trickfire/Software-USB-Communication-Commands-1651fd41ff5b80adbc37eacd546f46b2
 */
void updateStepperConfig(char* data, int arraySize){
    int port = data[13] - '0';
    int ms1 = data[15] - '0';
    int ms2 = data[17] - '0';
    int ms3 = data[19] - '0';

    //Debugging
    // printf("PORT %d\n", port);
    // printf("MS1 %d\n", ms1);
    // printf("MS2 %d\n", ms2);
    // printf("MS3 %d\n", ms3);

    // Lock the stepper semaphore
    SemaphoreHandle_t* semaphore = &stepperMutexes[port];

    if (xSemaphoreTake(*semaphore, 10)) {
        StepperMotor* stepper = &stepperMotors[port];
        stepper->MS1 = ms1;
        stepper->MS2 = ms2;
        stepper->MS3 = ms3;
        stepper->isMsDirty = true;

        // Unlock the stepper semaphore
        xSemaphoreGive(*semaphore);
    }
}

/**
 * Handles the "stepper power..." command
 * Look at the Notion docs for more info about the command definition:
 * https://www.notion.so/trickfire/Software-USB-Communication-Commands-1651fd41ff5b80adbc37eacd546f46b2
 */
void updateStepperPower(char* data, int arraySize){
    int port = data[14] - '0';
    int enable = data[16] - '0'; // integer 0 is 0; integer 1 is 1; integer 59 is k
    int sleep = data[18] - '0';
    int reset = data[20] - '0';

    //Debugging
    // printf("PORT %d\n", port);
    // printf("enable %d\n", enable);
    // printf("sleep %d\n", sleep);
    // printf("reset %d\n", reset);

    // lock the semaphore
    SemaphoreHandle_t* semaphore = &stepperMutexes[port];

    if (xSemaphoreTake(*semaphore, 10)) {
        StepperMotor* stepper = &stepperMotors[port];
        if (enable != 59) { // If "k" (for "keep"), then we do not do anything for this entry
            stepper->isEnable = enable;
            stepper->isEnableDirty = true;
        }

        if (sleep != 59) {
            stepper->isSleep = sleep;
            stepper->isSleepDirty = true;
        }

        if (reset != 59) {
            stepper->isReset = reset;
            stepper->isResetDirty = true;
        }

        // Unlock the sempahore
        xSemaphoreGive(*semaphore);
    }
}

/**
 * Handle the "pwm..." command
 * Look at the Notion docs for more info about the command definition:
 * https://www.notion.so/trickfire/Software-USB-Communication-Commands-1651fd41ff5b80adbc37eacd546f46b2
 */
void updateStepperPWM(char* data, int arraySize){
    int port = data[4] - '0';
    float dutyCycle = readAndConvertRawBitsIntoFloat(data, arraySize, 6, 38);
    float frequency = readAndConvertRawBitsIntoFloat(data, arraySize, 39, 71);

    // Debugging
    // printf("PORT %d\n", port);
    // printf("Duty Cycle %f\n", dutyCycle);
    // printf("Frequency %f\n", frequency);

    // Clamp the duty cycle to [0.0, 100.0]
    if (dutyCycle > 100.0) {
        dutyCycle = 1.0F;
    }
    else if (dutyCycle < 0.0) {
        dutyCycle = 0.0F;
    }

    // Update the servo motors. Use microseconds as it is common for
    // servo motors to use 0.5ms to 1.5ms for 0deg and 180deg
    // Thus it is better to do 500us to 1500us because they are integers

    Servo* servoMotor = &servos[port];

    float period = (1.0 / frequency) * 1000000.0; // seconds * 1000000 us

    // lock the semaphore
    SemaphoreHandle_t* semaphore = &servoMutexes[port];

    if (xSemaphoreTake(*semaphore, 10)) {

        servoMotor->onTime = (dutyCycle / 100.0) * period; // implicit float -> unint32_t
        servoMotor->offTime = (1.0 - (dutyCycle / 100.0)) * period;

        // Unlock servo semaphore
        xSemaphoreGive(*semaphore);
    }

}

//TODO finish this in the future
void updateStepperLIGHT(char* data, int arraySize){
    printf("LIGHT\n");

    char lightType = data[6];
    int state = data[8] - '0';

    printf("lightType %c\n", lightType);
    printf("state%d\n", state);

    //update the light directly here via the i2c
    

}

/**
 * Sends data to the host by handling the "getmbd..." command
 * There are three types: pwm, stepper, and light
 * 
 * Look at the Notion docs for more info about the command definition:
 * https://www.notion.so/trickfire/Software-USB-Communication-Commands-1651fd41ff5b80adbc37eacd546f46b2
 */
void sendMBDeviceData(char* data, int arraySize){
    char deviceID[3] = {};
    deviceID[0] = data[7];
    deviceID[1] = data[8];
    deviceID[2] = data[9];

    int port = data[11] - '0';

    if (deviceID[0] == 'p' && deviceID[1] == 'w' && deviceID[2] == 'm') {
        Servo* servoMotor = &servos[port];

        float period;
        float frequency;

        float dutyCycle;

        // lock the servo semaphore
        SemaphoreHandle_t* semaphore = &servoMutexes[port];

        if (xSemaphoreTake(*semaphore, 10)) {

            period = (servoMotor->onTime + servoMotor->offTime) / 1000000.0; //in in sec
            frequency = 1.0 / period;

            dutyCycle = (float)servoMotor->onTime / (servoMotor->onTime + servoMotor->offTime) * 100;

            xSemaphoreGive(*semaphore);
        }

        // Do funky pointer bit stuff to get the pure bits
        float* freqPtr = &frequency;
        u_int32_t* freqIntPtr = (u_int32_t*)freqPtr;

        float* dutyCyclePtr = &dutyCycle;
        u_int32_t* dutyCycleIntPtr = (u_int32_t*)dutyCyclePtr;

        // Lock writing to usb uart mutex
        SemaphoreHandle_t* usbSemaphore = &writeToUsbMutex;

        if (xSemaphoreTake(*usbSemaphore, 10)) {
            printf("pwm ");

            for (int bitShift = 31; bitShift > -1; bitShift--) {
                int bit = ((*dutyCycleIntPtr) & (1 << bitShift)) >> bitShift;

                printf("%u", bit);
            }

            printf(" ");

            for (int bitShift = 31; bitShift > -1; bitShift--) {
                int bit = ((*freqIntPtr) & (1 << bitShift)) >> bitShift;

                printf("%u", bit);
            }

            printf("\n");

            // unlock writing to usb uart mutex
            xSemaphoreGive(*usbSemaphore);
        }
        
        
    }
    else if (deviceID[0] == 's' && deviceID[1] == 't' && deviceID[2] == 'p') {
        StepperMotor* stepper = &stepperMotors[port];

        // lock the semaphore
        SemaphoreHandle_t* semaphore = &stepperMutexes[port];

        StepResolution stepResolution;
        float totalStepsPerRev;
        float position;
        float velocity;
        bool ignoreTargetPos;

        if (xSemaphoreTake(*semaphore, 10)) {
            stepResolution = getStepperMotorStepRes(port);
            totalStepsPerRev = 200 * stepResolution;

            position = stepper->targetPosition / totalStepsPerRev;
            velocity = 1000000.0 / (totalStepsPerRev * stepper->stepInterval);


            if (stepper->dir == 0) {
                velocity *= -1;
            }

            ignoreTargetPos = stepper->ignoreTargetPos;


            xSemaphoreGive(*semaphore);
        }

        // Lock writing to usb uart mutex
        SemaphoreHandle_t* usbSemaphore = &writeToUsbMutex;

        if (xSemaphoreTake(*usbSemaphore, 10)) {
            // --- funky pointer stuff
            float* posPtr = &position;
            u_int32_t* posIntPtr = (u_int32_t*)posPtr;

            float* velPtr = &velocity;
            u_int32_t* velIntPtr = (u_int32_t*)velPtr;

            // lock the write out mutex
            printf("stp ");

            for (int bitShift = 31; bitShift > -1; bitShift--) {
                int bit = ((*velIntPtr) & (1 << bitShift)) >> bitShift;

                printf("%u", bit);
            }

            printf(" ");

            for (int bitShift = 31; bitShift > -1; bitShift--) {
                int bit = ((*posIntPtr) & (1 << bitShift)) >> bitShift;

                printf("%u", bit);
            }

            printf(" %u", ignoreTargetPos);

            printf("\n");
            // unlock writing to usb uart mutex
            xSemaphoreGive(*usbSemaphore);
        }




    }
    else { // "lgh"
        printf("Return light data\n");
        //TODO finish this when light data is available
    }


}

/**
 * For debugging, handles the "getmbb..." command
 * Unlike other commands, this does not care about sending the correct
 * IEEE 754 floats or 32-bit integers as this is supposed to be
 * printed to a console, not handled as actual data
 * 
 * Look at the Notion docs for more info about the command definition:
 * https://www.notion.so/trickfire/Software-USB-Communication-Commands-1651fd41ff5b80adbc37eacd546f46b2
 */
void sendMBDebugDeviceData(char* data, int arraySize){
    char deviceID[3] = {};
    deviceID[0] = data[7];
    deviceID[1] = data[8];
    deviceID[2] = data[9];

    int port = data[11] - '0';

    // Lock writing to usb uart mutex
    SemaphoreHandle_t* usbSemaphore = &writeToUsbMutex;

    if (xSemaphoreTake(*usbSemaphore, 10)) {
        // lock the semaphore

        if (deviceID[0] == 'p' && deviceID[1] == 'w' && deviceID[2] == 'm') {
            Servo* servoMotor = &servos[port];

            SemaphoreHandle_t* semaphore = &servoMutexes[port];
            if (xSemaphoreTake(*semaphore, 10)) {
                printf("%d %d %d %d\n",
                servoMotor->isOn,
                servoMotor->startTime,
                servoMotor->onTime,
                servoMotor->offTime);

                xSemaphoreGive(*semaphore);
            }
        }
        else if (deviceID[0] == 's' && deviceID[1] == 't' && deviceID[2] == 'p') {
            StepperMotor* stepper = &stepperMotors[port];
            SemaphoreHandle_t* semaphore = &stepperMutexes[port];

            if (xSemaphoreTake(*semaphore, 10)) {

                printf("%d %d %d %d %d %d %d %d %d %d %d %d\n", 
                    stepper->isEnable,
                    stepper->isSleep,
                    stepper->isReset,
                    stepper->ignoreTargetPos,
                    stepper->expanderAddr,
                    stepper->MS1,
                    stepper->MS2,
                    stepper->MS3,
                    stepper->dir,
                    stepper->currentPosition,
                    stepper->targetPosition,
                    stepper->stepInterval);

                xSemaphoreGive(*semaphore);
            }

        }
        else { // "lgh"
            //TODO finish the light data
            // I am waiting for when this data will be available
        }

        // unlock writing to usb uart mutex
        xSemaphoreGive(*usbSemaphore);
    }
}

/**
 * Convert raw bits from a string array into a IEEE 754 float
 */
float readAndConvertRawBitsIntoFloat(char* data, int arraySize, int start, int end){

    float finalNumber = 0;

    // --- Converting the finalNumber into its IEEE 754 floating point number ---

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

/**
 * Returns the type of step resolution the stepper motor
 * controller is using
 * 
 * returns: entry from the StepResolution
 */
StepResolution getStepperMotorStepRes(uint8_t port){
    StepperMotor stepper = stepperMotors[port];

    uint8_t ms1 = stepper.MS1;
    uint8_t ms2 = stepper.MS2;
    uint8_t ms3 = stepper.MS3;

    if (ms1 == 0 && ms2 == 0 && ms3 == 0) {
        return StepResolution::FULL;
    }
    else if (ms1 == 1 && ms2 == 0 && ms3 == 0) {
        return StepResolution::HALF;
    }
    else if (ms1 == 0 && ms2 == 1 && ms3 == 0) {
        return StepResolution::QUARTER;
    }
    else if (ms1 == 1 && ms2 == 1 && ms3 == 0) {
        return StepResolution::EIGHTH;
    }
    else { // ms1 == 1 && ms2 == 1 && ms3 == 1
        return StepResolution::SIXTEENTH;
    }
}