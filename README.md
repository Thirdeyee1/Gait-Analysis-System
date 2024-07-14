# Dynamic Gait Cycle Calculation using Arduino BLE 

This repository contains the code and documentation for a dynamic gait cycle calculator using Arduino and Bluetooth Low Energy (BLE). Unlike traditional gait cycle calculators, which require a fixed number of steps or a specific time duration to calculate cadence, this system can compute cadence and other parameters dynamically, even after just a few steps.

## Features

- **Dynamic Cadence Calculation:** Calculate cadence based on real-time data, without needing a fixed number of steps.
- **Total Steps:** Track the total number of steps taken.
- **Step Count:** Separate counts for left and right steps.
- **Step Rate:** Calculate step rate for both left and right steps.
- **Walking Speed:** Determine walking speed based on accelerometer data.
- **Step Length:** Calculate the length of each step.
- **Stride Length:** Determine the stride length by summing the left and right step lengths.
- **Cadence:** Calculate cadence in steps per minute.

## Formulas

- **Left Current Step Rate**: `LeftCurrentStepRate = leftStepsCount / elapsedTime`
- **Right Current Step Rate**: `RightCurrentStepRate = rightStepsCount / elapsedTime`
- **Walking Speed (m/s)**:
  ```cpp
  Magnitude = sqrt(Xaxis^2 + Yaxis^2 + Zaxis^2);
  deltaTime = 0.01; // 10ms time interval to update based on accelerometer changes
  decayRate = 0.995; // multiplier when no acceleration or motion detected
  WalkingSpeed = WalkingSpeed + Magnitude * deltaTime;
  // When no motion:
  WalkingSpeed = WalkingSpeed * decayRate;
  ```
- **Left Step Length (m)**: `leftStepLength = WalkingSpeed / LeftCurrentStepRate`
- **Right Step Length (m)**: `rightStepLength = WalkingSpeed / RightCurrentStepRate`
- **Stride Length (m)**: `StrideLength = leftStepLength + rightStepLength`
- **Cadence (spm)**: `Cadence = (WalkingSpeed / StrideLength) * 60`

## Hardware Requirements

- Arduino Nano 33 IoT or Arduino Nano 33 BLE
- Force-sensitive resistors (FSR) for left and right feet
- Accelerometer for hip motion detection
- BLE peripherals for left foot, right foot, and hip

## Software Requirements

- Arduino IDE
- ArduinoBLE library (install via Arduino Library Manager)

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/Dynamic-Gait-Cycle-Calculator.git
    ```
2. Open the Arduino IDE and load the `GaitCycleCalculator.ino` file from the `src` directory.
3. Install the ArduinoBLE library via the Library Manager in the Arduino IDE.
4. Upload the code to your Arduino board.

## Usage

1. Power on the Arduino and ensure that all peripherals are properly connected.
2. Open the Serial Monitor to view the output data.
3. Start walking and observe the dynamic calculation of gait parameters in real-time.

## Code Structure

- `GaitCycleCalculator.ino`: Main file containing setup and loop functions.
- `BLEHandler.h` and `BLEHandler.cpp`: Handles BLE communication.
- `GaitCalculation.h` and `GaitCalculation.cpp`: Contains functions for gait cycle calculations.
- `Utils.h` and `Utils.cpp`: Utility functions for data conversion and processing.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
