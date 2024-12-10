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
- Gyroscope Sensor (e.g., MPU6050)
- C++ Compiler (compatible with Mbed OS)
- LED indicators and push buttons.

## Setup Instructions
1. Clone the repository:
        ```bash
        git clone https://github.com/<your-username>/gesture-recognition-system.git
        cd gesture-recognition-system
        ```
2. Compile the code using the Mbed framework.
3. Flash the compiled binary to the target hardware.
4. Connect the gyroscope, LEDs, and push button as per the circuit diagram.
5. Power up the system and follow the instructions in the terminal or LEDs.

## Usage
1. Press the button to start recording a gesture.
2. Move the device in the desired pattern.
3. Release the button to end recording.
4. The system compares the gesture with a reference and provides LED feedback.

## Contributors
- Bibek Poudel
- Rugved Mahtre
- Akshay Parihalkar

## License
This project is licensed under the MIT License.esture Recognition System
