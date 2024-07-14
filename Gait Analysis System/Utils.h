#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

int getIntValue(const unsigned char* data) {
    int value;
    memcpy(&value, data, sizeof(value));
    return value;
}

float byteArrayToFloat(const unsigned char* byteArray) {
    float value;
    memcpy(&value, byteArray, sizeof(float));
    return value;
}

int byteArrayToInt(const unsigned char* byteArray) {
    int value;
    memcpy(&value, byteArray, sizeof(int));
    return value;
}

#endif // UTILS_H
