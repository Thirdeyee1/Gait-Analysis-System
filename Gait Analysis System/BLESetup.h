#ifndef BLE_SETUP_H
#define BLE_SETUP_H

#include <ArduinoBLE.h>
#include "GaitCalculation.h"
#include "Utils.h"

// Define constants and variables
#define BLE_MAX_PERIPHERALS 3
#define BLE_SCAN_INTERVAL 10000
#define BLE_SCAN_NEW_DEVICES 10000

BLEDevice peripherals[BLE_MAX_PERIPHERALS];
bool peripheralsConnected[BLE_MAX_PERIPHERALS] = { 0 };
bool peripheralsToConnect[BLE_MAX_PERIPHERALS] = { 0 };
int peripheralCounter = 0;
unsigned long control_time;
bool ok = true;

// BLE characteristics and service
BLECharacteristic hipAccelerometerDataCharacteristic[BLE_MAX_PERIPHERALS];
BLECharacteristic angleOrientationCharacteristic[BLE_MAX_PERIPHERALS];
BLECharacteristic fsrDataCharacteristic[BLE_MAX_PERIPHERALS];
BLEService gait("19b10014-e8f2-537e-4f6c-d104768a1214");

BLECharacteristic resetCharacteristic("19b10030-e8f2-537e-4f6c-d104768a1219", BLEWrite, 1);
BLECharacteristic startCharacteristic("19b10031-e8f2-537e-4f6c-d104768a1219", BLEWrite, 1);
BLECharacteristic stepCountCharacteristic("19b10017-e8f2-537e-4f6c-d104768a1221", BLENotify, 20);
BLEFloatCharacteristic walkingSpeedCharacteristic("19b10090-e8f2-537e-4f6c-d104768a1215", BLENotify);
BLEFloatCharacteristic leftStepLengthCharacteristic("19b10091-e8f2-537e-4f6c-d104768a1215", BLENotify);
BLEFloatCharacteristic rightStepLengthCharacteristic("19b10092-e8f2-537e-4f6c-d104768a1215", BLENotify);
BLEFloatCharacteristic strideLengthCharacteristic("19b10093-e8f2-537e-4f6c-d104768a1215", BLENotify);
BLEFloatCharacteristic cadenceCharacteristic("19b10094-e8f2-537e-4f6c-d104768a1215", BLENotify);

void setupBLE() {
    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        while (1);
    }
    Serial.println("Bluetooth Central - Sensor Data");

    BLE.setLocalName("Nano 33 IoT (Central)");
    BLE.setAdvertisedService(gait);
    gait.addCharacteristic(stepCountCharacteristic);
    gait.addCharacteristic(walkingSpeedCharacteristic);
    gait.addCharacteristic(leftStepLengthCharacteristic);
    gait.addCharacteristic(rightStepLengthCharacteristic);
    gait.addCharacteristic(strideLengthCharacteristic);
    gait.addCharacteristic(cadenceCharacteristic);
    gait.addCharacteristic(resetCharacteristic);
    gait.addCharacteristic(startCharacteristic);
    BLE.addService(gait);

    BLE.advertise();
    resetCharacteristic.subscribe();
    startCharacteristic.subscribe();
    peripheralCounter = 0;
}

void loopBLE() {
    BLE.scanForUuid("19b10010-e8f2-537e-4f6c-d104768a1214");
    Serial.println("Scan ongoing");
    unsigned long startMillis = millis();
    while (millis() - startMillis < BLE_SCAN_INTERVAL && peripheralCounter < BLE_MAX_PERIPHERALS) {
        BLEDevice peripheral = BLE.available();
        if (peripheral) {
            if (peripheral.localName() == "Left_Foot_Peripheral" && !peripheralsToConnect[0] && !peripheralsConnected[0]) {
                peripherals[0] = peripheral;
                peripheralCounter++;
                peripheralsToConnect[0] = true;
            }
            if (peripheral.localName() == "Right_Foot_Peripheral" && !peripheralsToConnect[1] && !peripheralsConnected[1]) {
                peripherals[1] = peripheral;
                peripheralCounter++;
                peripheralsToConnect[1] = true;
            }
            if (peripheral.localName() == "Hip_Peripheral" && !peripheralsToConnect[2] && !peripheralsConnected[2]) {
                peripherals[2] = peripheral;
                peripheralCounter++;
                peripheralsToConnect[2] = true;
            }
        }
    }

    Serial.print("Device found: ");
    Serial.println(peripheralCounter);

    BLE.stopScan();

    for (int i = 0; i < BLE_MAX_PERIPHERALS; i++) {
        if (peripheralsToConnect[i]) {
            peripherals[i].connect();
            peripherals[i].discoverAttributes();

            BLEService imuService = peripherals[i].service("19b10010-e8f2-537e-4f6c-d104768a1214");
            hipAccelerometerDataCharacteristic[i] = imuService.characteristic("19b10013-e8f2-537e-4f6c-d104768a1215");
            angleOrientationCharacteristic[i] = imuService.characteristic("19b10011-e8f2-537e-4f6c-d104768a1215");
            fsrDataCharacteristic[i] = imuService.characteristic("19b10050-e8f2-537e-4f6c-d104768a1216");

            if (hipAccelerometerDataCharacteristic[i]) {
                hipAccelerometerDataCharacteristic[i].subscribe();
            }

            if (angleOrientationCharacteristic[i]) {
                angleOrientationCharacteristic[i].subscribe();
            }

            if (fsrDataCharacteristic[i]) {
                fsrDataCharacteristic[i].subscribe();
            }

            peripheralsConnected[i] = true;
            peripheralsToConnect[i] = false;
        }
    }

    control_time = millis();
    ok = true;

    while (ok) {
        if (peripheralCounter < BLE_MAX_PERIPHERALS) {
            if (millis() - control_time > BLE_SCAN_NEW_DEVICES) {
                ok = false;
                Serial.println("Looking for other devices");
            }
        }

        for (int i = 0; i < BLE_MAX_PERIPHERALS; i++) {
            if (peripheralsConnected[i]) {
                if (!peripherals[i].connected()) {
                    ok = false;
                    peripheralsConnected[i] = false;
                    peripheralCounter--;
                    Serial.print("Device ");
                    Serial.print(i);
                    Serial.println(" disconnected.");
                }
                else {
                    handleCharacteristicUpdates(i);
                }
            }
        }
    }
}

#endif // BLE_SETUP_H
