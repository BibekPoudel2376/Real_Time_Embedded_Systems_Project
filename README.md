# Gesture Recognition System

## Overview
This project implements a gesture recognition system using a gyroscope sensor and LED indicators. The system captures gestures, maps them into directional values, and compares them against reference gestures for recognition.

The code is written in C++ using the Mbed framework and demonstrates the integration of hardware components like gyroscopes, LEDs, and buttons through SPI communication.

## Features
- **Gesture Capture**: Records real-time gesture data using a gyroscope sensor.
- **LED Feedback**: Indicates recognition success or failure.
- **Calibration**: Adjusts sensor readings for accuracy.
- **Gesture Mapping**: Converts sensor data into meaningful directional values.
- **Gesture Comparison**: Matches input gestures with predefined reference gestures.

## System Components
- **Gyroscope Sensor**: Captures motion data in three axes (x, y, z).
- **LED Indicators**: Provide visual feedback (e.g., recognition success/failure).
- **Push Button**: Initiates gesture recording.
- **SPI Communication**: Facilitates data transfer between components.
- **Calibration**: Ensures sensor accuracy by adjusting for offsets.

## Code Implementation
The project consists of the following:

- **Libraries and Constants**: Predefined values for calibration, sampling, and processing.
- **Global Variables**: Stores sensor data, gesture maps, and system states.
- **Functions**:
    - `record_gesture_data()`: Records sensor data during gestures.
    - `map_gesture_data()`: Maps raw data to directional values.
    - `compare_gesture()`: Compares recorded gestures to reference gestures.
    - `calibrate_sensor()`: Calibrates the gyroscope sensor for accuracy.
- **Main Loop**: Manages gesture recording, mapping, and comparison in real-time.

## Results
- **Accuracy**: ~90% success rate in recognizing predefined gestures.
- **Robustness**: Handles variations in gesture speed and minor delays.

### Limitations
- Struggles with extremely fast or slow gestures.
- Calibration process can be improved for environmental variability.

## Future Improvements
- Enhance calibration for diverse environments.
- Support more complex gestures using additional sensors.
- Develop a user-friendly interface for configuration and calibration.
- Expand gesture mapping to accommodate wider speed ranges.

## Prerequisites
- Mbed Framework
- STM32F49I Discovery Board
- C++ Compiler (compatible with Mbed OS)

## Setup Instructions
1. Clone the repository:
        ```bash
        git clone https://github.com/bk-poudel/gesture-recognition-system.git
        cd gesture-recognition-system
        ```
2. Compile the code using the Mbed framework.
3. Flash the compiled binary to the target hardware.

## Usage
1. The system records the gesture while booting up. So, you can record new gesture by pressing the reset button
2. To perform the gesture press blue button (in stm32f49i board), and strat performing the gesture
3. If the gesture matches the red led will turn off, otherwise the red LED will be on.

## Contributors
- Bibek Poudel
- Rugved Mahtre
- Akshay Parihalkar

## License
This project is licensed under the MIT License. Gesture Recognition System
