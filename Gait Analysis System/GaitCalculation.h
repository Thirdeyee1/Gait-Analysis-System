#pragma once
#ifndef GAIT_CALCULATION_H
#define GAIT_CALCULATION_H

#include <ArduinoBLE.h>
#include "Utils.h"

// BLE Characteristics and Services
extern BLECharacteristic hipAccelerometerDataCharacteristic[BLE_MAX_PERIPHERALS];
extern BLECharacteristic angleOrientationCharacteristic[BLE_MAX_PERIPHERALS];
extern BLECharacteristic fsrDataCharacteristic[BLE_MAX_PERIPHERALS];
extern BLECharacteristic stepCountCharacteristic;
extern BLEFloatCharacteristic walkingSpeedCharacteristic;
extern BLEFloatCharacteristic leftStepLengthCharacteristic;
extern BLEFloatCharacteristic rightStepLengthCharacteristic;
extern BLEFloatCharacteristic strideLengthCharacteristic;
extern BLEFloatCharacteristic cadenceCharacteristic;
extern BLECharacteristic resetCharacteristic;
extern BLECharacteristic startCharacteristic;

// Gait calculation parameters
unsigned long startTime; // Start time of gait calculation
float left_step_length = 0; // Length of the left step
float right_step_length = 0; // Length of the right step
float stride_length = 0; // Combined length of the left and right steps (stride length)
int left_fsr; // Force sensor reading for the left foot
int right_fsr; // Force sensor reading for the right foot
const float STEP_THRESHOLD = 120; // Threshold for detecting steps based on yaw
bool isGaitCalculationStarted = false; // Flag to check if gait calculation has started
int stepCount = 0; // Total step count
unsigned long lastStepTime = 0; // Timestamp of the last detected step
const unsigned long stepDelay = 700; // Minimum delay between steps to avoid false positives
bool stepDetectedLeft = false; // Flag to check if a step is detected for the left foot
bool stepDetectedRight = false; // Flag to check if a step is detected for the right foot
float leftCurrentStepRate = 0; // Current step rate for the left foot
float rightCurrentStepRate = 0; // Current step rate for the right foot
float leftStepsCount = 0; // Total steps counted for the left foot
float rightStepsCount = 0; // Total steps counted for the right foot
float deltaTime = 0.01; // Time interval for updating walking speed based on accelerometer data
float decayRate = 0.995; // Decay rate for walking speed when no motion is detected
float walkingSpeed = 0; // Calculated walking speed in m/s
float cadence = 0; // Calculated cadence in steps per minute
float magnitude = 0; // Magnitude of acceleration
float hipAccX, hipAccY, hipAccZ; // Accelerometer readings for the hip
float roll, pitch, yaw; // Orientation angles

// Function to handle characteristic updates
void handleCharacteristicUpdates(int index) {
    if (angleOrientationCharacteristic[index] && angleOrientationCharacteristic[index].valueUpdated()) {
        // Read orientation data (roll, pitch, yaw)
        const unsigned char* angleDataBytes = angleOrientationCharacteristic[index].value();
        roll = byteArrayToFloat(angleDataBytes);
        pitch = byteArrayToFloat(angleDataBytes + sizeof(float));
        yaw = byteArrayToFloat(angleDataBytes + 2 * sizeof(float));

        // Read force sensor readings for left and right foot
        if (index == 0) {
            if (fsrDataCharacteristic[0] && fsrDataCharacteristic[0].valueUpdated()) {
                const unsigned char* fsrDataBytes = fsrDataCharacteristic[0].value();
                left_fsr = getIntValue(fsrDataBytes);
            }
        }
        else if (index == 1) {
            if (fsrDataCharacteristic[1] && fsrDataCharacteristic[1].valueUpdated()) {
                const unsigned char* fsrDataBytes = fsrDataCharacteristic[1].value();
                right_fsr = getIntValue(fsrDataBytes);
            }
        }

        // Read accelerometer data from hip
        if (hipAccelerometerDataCharacteristic[2] && hipAccelerometerDataCharacteristic[2].valueUpdated()) {
            const unsigned char* accDataBytes = hipAccelerometerDataCharacteristic[2].value();
            hipAccX = byteArrayToFloat(accDataBytes);
            hipAccY = byteArrayToFloat(accDataBytes + sizeof(float));
            hipAccZ = byteArrayToFloat(accDataBytes + 2 * sizeof(float));
        }

        // Print orientation and accelerometer data for debugging
        Serial.print("Roll: ");
        Serial.println(roll);
        Serial.print("Pitch: ");
        Serial.println(pitch);
        Serial.print("Yaw: ");
        Serial.println(yaw);
        Serial.print("HipAccX: ");
        Serial.print(hipAccX);
        Serial.print(" HipAccY: ");
        Serial.print(hipAccY);
        Serial.print(" HipAccZ: ");
        Serial.println(hipAccZ);

        // Start gait calculation if the flag is set
        if (isGaitCalculationStarted) {
            unsigned long currentTime = millis();
            // Detect steps based on force sensor readings and yaw threshold
            if ((left_fsr >= 200 || right_fsr >= 200) && yaw < STEP_THRESHOLD) {
                if (index == 0) {
                    if (!stepDetectedLeft && currentTime - lastStepTime >= stepDelay) {
                        stepDetectedLeft = true;
                        stepCount++;
                        String stepCountData = String(stepCount);
                        stepCountCharacteristic.writeValue(stepCountData.c_str());
                        leftStepsCount++;
                        unsigned long elapsedTime = currentTime - startTime;
                        if (elapsedTime > 0) {
                            leftCurrentStepRate = leftStepsCount / ((float)elapsedTime / 1000);
                        }
                        lastStepTime = currentTime;
                    }
                }
                else if (index == 1) {
                    if (!stepDetectedRight && currentTime - lastStepTime >= stepDelay) {
                        stepDetectedRight = true;
                        stepCount++;
                        String stepCountData = String(stepCount);
                        stepCountCharacteristic.writeValue(stepCountData.c_str());
                        rightStepsCount++;
                        unsigned long elapsedTime = currentTime - startTime;
                        if (elapsedTime > 0) {
                            rightCurrentStepRate = rightStepsCount / ((float)elapsedTime / 1000);
                        }
                        lastStepTime = currentTime;
                    }
                }
            }
            else {
                if (index == 0) {
                    stepDetectedLeft = false;
                }
                else if (index == 1) {
                    stepDetectedRight = false;
                }
            }

            // Calculate walking speed based on accelerometer data
            magnitude = sqrt(hipAccX * hipAccX + hipAccY * hipAccY + hipAccZ * hipAccZ);
            if (magnitude > 1.10) {
                walkingSpeed += magnitude * deltaTime;
            }
            else {
                walkingSpeed *= decayRate;
            }
            walkingSpeed = max(0.0, walkingSpeed);
            walkingSpeedCharacteristic.writeValue(walkingSpeed);

            // Calculate step lengths and stride length
            if (leftCurrentStepRate != 0) {
                left_step_length = walkingSpeed / leftCurrentStepRate;
            }
            if (rightCurrentStepRate != 0) {
                right_step_length = walkingSpeed / rightCurrentStepRate;
            }
            stride_length = left_step_length + right_step_length;

            // Calculate cadence
            if (stride_length > 0) {
                cadence = (walkingSpeed * 60) / stride_length;
            }
            else {
                cadence = 0;
            }

            // Write calculated values to BLE characteristics
            leftStepLengthCharacteristic.writeValue(left_step_length);
            rightStepLengthCharacteristic.writeValue(right_step_length);
            strideLengthCharacteristic.writeValue(stride_length);
            cadenceCharacteristic.writeValue(cadence);
        }
    }

    // Handle reset characteristic
    if (resetCharacteristic.written()) {
        bool resetValue = resetCharacteristic.value();
        if (resetValue) {
            isGaitCalculationStarted = false;
            stepCount = 0;
            walkingSpeed = 0;
            left_step_length = 0;
            right_step_length = 0;
            stride_length = 0;
            leftCurrentStepRate = 0;
            rightCurrentStepRate = 0;
            leftStepsCount = 0;
            rightStepsCount = 0;
            Serial.println("Gait calculation reset to zero");
        }
    }

    // Handle start characteristic
    if (startCharacteristic.written()) {
        bool startValue = startCharacteristic.value();
        if (startValue) {
            isGaitCalculationStarted = true;
            startTime = millis();
            Serial.println("Gait calculation started");
        }
        else {
            isGaitCalculationStarted = false;
            stepCount = 0;
            walkingSpeed = 0;
            left_step_length = 0;
            right_step_length = 0;
            stride_length = 0;
            leftCurrentStepRate = 0;
            rightCurrentStepRate = 0;
            leftStepsCount = 0;
            rightStepsCount = 0;
            Serial.println("Gait calculation stopped");
        }
    }
}

#endif // GAIT_CALCULATION_H
