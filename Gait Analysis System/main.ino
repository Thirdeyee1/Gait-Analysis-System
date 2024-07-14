#include "BLESetup.h"
#include "GaitCalculation.h"
#include "Utils.h"

void setup() {
    Serial.begin(9600);
    setupBLE();
}

void loop() {
    loopBLE();
}
