#include "mbed.h"
#include "sensor.h"
#include "led.h"
#include <vector>
#include <map>
#include <algorithm>
//Final Working Code 
//Calibration and Analysis
const int WINDOW_SIZE = 5;
std::vector<float> window_x(WINDOW_SIZE), window_y(WINDOW_SIZE), window_z(WINDOW_SIZE);
int window_index = 0;
const int CALIBRATION_SAMPLES = 100;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
bool analysis_required=false;
// SPI pins
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // SPI instance
uint8_t write_buf[32], read_buf[32];

// LED pins
LEDController led_controller(LED2, LED1);

// Button for recording
InterruptIn button(BUTTON1);

// Constants for recording
const int MAX_SAMPLES = 200;               // Maximum number of samples
const int RECORDING_DURATION_MS = 3000;    // Recording duration in milliseconds
const int SAMPLING_INTERVAL_MS = RECORDING_DURATION_MS / MAX_SAMPLES; // Interval between samples

// Buffers for gesture data
float recorded_gesture_data[MAX_SAMPLES][3]; // Buffer for reference gesture
float gesture_data[MAX_SAMPLES][3];          // Buffer for test gesture

// Timer and Variables
Timer sample_timer;   // Timer for sampling intervals
int sample_count = 0; // Sample counter
volatile bool recording = false; // Flag to indicate recording state
volatile bool key_recording = true; // Flag to indicate recording state
float key_gesture_map[20][3]; // Buffer for gesture mapping
int gesture_map_window_size =30;
float recorded_gesture_map[20][3]; // Buffer for gesture mapping
//Function Declarations
void record_key();
void record_gesture_data();
void map_gesture_data(float gesture_data[MAX_SAMPLES][3], float threshold, int window_size, float gesture_map[20][3]);
void print_data(float buffer[MAX_SAMPLES][3]);
bool compare_gesture(float gesture_data[20][3], float gesture_map[20][3]);

// Function to find mode
float find_mode(std::vector<float>& data) {
    std::map<float, int> frequency_map;
    for (float value : data) {
        frequency_map[value]++;
    }
    return std::max_element(frequency_map.begin(), frequency_map.end(), 
                            [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                                return a.second < b.second;
                            })->first;
}

// Function to find median
float find_median(std::vector<float>& data) {
    std::sort(data.begin(), data.end());
    if (data.size() % 2 == 0) {
        return (data[data.size() / 2 - 1] + data[data.size() / 2]) / 2.0;
    } else {
        return data[data.size() / 2];
    }
}

// Function to calibrate sensor
void calibrate_sensor() {
    std::vector<float> gx_samples(CALIBRATION_SAMPLES), gy_samples(CALIBRATION_SAMPLES), gz_samples(CALIBRATION_SAMPLES);
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        float gx, gy, gz;
        read_sensor_data(spi, write_buf, read_buf, gx, gy, gz);
        gx_samples[i] = gx;
        gy_samples[i] = gy;
        gz_samples[i] = gz;
        thread_sleep_for(10); // Adjust delay as needed
    }
    gx_offset = find_mode(gx_samples);
    gy_offset = find_mode(gy_samples);
    gz_offset = find_mode(gz_samples);
    printf("Calibration complete: gx_offset=%f, gy_offset=%f, gz_offset=%f\n", gx_offset, gy_offset, gz_offset);
}

// Function to apply median filter
void apply_median_filter(float& gx, float& gy, float& gz) {
    window_x[window_index] = gx;
    window_y[window_index] = gy;
    window_z[window_index] = gz;
    window_index = (window_index + 1) % WINDOW_SIZE;

    std::vector<float> temp_x = window_x;
    std::vector<float> temp_y = window_y;
    std::vector<float> temp_z = window_z;

    gx = find_median(temp_x);
    gy = find_median(temp_y);
    gz = find_median(temp_z);
}

//Printing data
void print_data(float buffer[MAX_SAMPLES][3]) {
    for (int i = 0; i < MAX_SAMPLES; ++i) {
        printf(">gx:%f\n", buffer[i][0]);
        printf(">gy:%f\n", buffer[i][1]);
        printf(">gz:%f\n", buffer[i][2]);
    }
}
// Function to record data into the target buffer
void record_key() {
    sample_timer.start();
    if(sample_timer.read_ms() >= SAMPLING_INTERVAL_MS) {
        if(sample_count < MAX_SAMPLES) {
            float gx, gy, gz;
            read_sensor_data(spi, write_buf, read_buf, gx, gy, gz);
            gx-=gx_offset;
            gy-=gy_offset;
            gz-=gz_offset;
            apply_median_filter(gx, gy, gz);
            recorded_gesture_data[sample_count][0] = gx;
            recorded_gesture_data[sample_count][1] = gy;
            recorded_gesture_data[sample_count][2] = gz;
            sample_count++;
            sample_timer.reset();
            led_controller.turn_on_green();
        } else {
            key_recording = false;
            led_controller.turn_off_green();
            sample_count = 0;
            sample_timer.stop();
            map_gesture_data(recorded_gesture_data, 15, 30, key_gesture_map);
            print_data(key_gesture_map);
        }
    }
}

void record_gesture_data() {
    sample_timer.start();
    if (recording) {
        if(sample_timer.read_ms() >= SAMPLING_INTERVAL_MS) {
            if(sample_count < MAX_SAMPLES) {
                float gx, gy, gz;
                read_sensor_data(spi, write_buf, read_buf, gx, gy, gz);
                            gx-=gx_offset;
            gy-=gy_offset;
            gz-=gz_offset;
            apply_median_filter(gx, gy, gz);
                gesture_data[sample_count][0] = gx;
                gesture_data[sample_count][1] = gy;
                gesture_data[sample_count][2] = gz;
                sample_count++;
                sample_timer.reset();
                led_controller.turn_on_green();
            } else {
                recording = false;
                led_controller.turn_off_green();
                sample_count = 0;
                sample_timer.stop();
                map_gesture_data(gesture_data, 15, 30, recorded_gesture_map);
                print_data(recorded_gesture_map);
            }
        }
    }
}


// Timer for adjustment time
Timer adjustment_timer;
bool adjustment_time_elapsed = false;

// ISR for button press (start recording)
void button_pressed_isr() {
    recording = true;
    adjustment_timer.reset();
    adjustment_timer.start();
    adjustment_time_elapsed = false;
    led_controller.turn_on_red();
    analysis_required=true;
}

// Main function
int main() {
    analysis_required=false;
    led_controller.turn_on_red();
    led_controller.turn_off_green();
    key_recording = true;
    recording = false;
    // Initialize SPI
    init_spi(spi, write_buf, read_buf);
    thread_sleep_for(1500);
    // Calibrate sensor
    //calibrate_sensor();
    //thread_sleep_for(5000);
    // Start the sample timer once, not resetting it until necessary
    
    // Attach the ISR to the button press event
    button.fall(&button_pressed_isr); // Use falling edge for button press
    while (1) {
        // Call the appropriate function based on the recording state
        if (key_recording) {
            record_key();
        } else {
            if (recording) {
                if (!adjustment_time_elapsed) {
                    if (adjustment_timer.read_ms() >= 1500) {
                        adjustment_time_elapsed = true;
                        adjustment_timer.stop();
                    }
                } else {
                    record_gesture_data();
                }
            }
        }
        if (!key_recording && !recording && analysis_required) {
            if (compare_gesture(key_gesture_map, recorded_gesture_map)) {
                led_controller.turn_off_red();
                analysis_required=false;
            }
        }
    }
}

//Create a function to crete a sliding window of size 20, take average of that window and then compare it with the threshold value. If the average value is greater than +threshold value for particular axis map it as 1, if it is less than -threshold value for particular axis map it as -1 and if it is between -threshold and +threshold map it as 0.
void map_gesture_data(float gesture_data[MAX_SAMPLES][3], float threshold, int window_size, float gesture_map[20][3]) {
    int window_start = 0;
    int window_end = window_size;
    int gesture_map_counter = 0;

    while (window_end <= MAX_SAMPLES) {
        float gx_sum = 0, gy_sum = 0, gz_sum = 0;

        for (int i = window_start; i < window_end; i++) {
            gx_sum += gesture_data[i][0];
            gy_sum += gesture_data[i][1];
            gz_sum += gesture_data[i][2];
        }

        float gx_avg = gx_sum / window_size;
        float gy_avg = gy_sum / window_size;
        float gz_avg = gz_sum / window_size;

        gesture_map[gesture_map_counter][0] = (gx_avg > threshold) ? 1 : (gx_avg < -threshold) ? -1 : 0;
        gesture_map[gesture_map_counter][1] = (gy_avg > threshold) ? 1 : (gy_avg < -threshold) ? -1 : 0;
        gesture_map[gesture_map_counter][2] = (gz_avg > threshold) ? 1 : (gz_avg < -threshold) ? -1 : 0;

        window_start += window_size / 2;
        window_end += window_size / 2;
        gesture_map_counter++;
    }
}
//The gesture mapped data are -1,1,0 so use a simple algorithm to account for delay and check if the two mapped data are same or not. If they are same then the gesture is same. If they are not same then the gesture is different.
bool compare_gesture(float gesture_map1[20][3], float gesture_map2[20][3]) {
    int false_counter=0;
    for (int i = 0; i < 20; i++) {
        if (gesture_map1[i][0] != gesture_map2[i][0] || gesture_map1[i][1] != gesture_map2[i][1] || gesture_map1[i][2] != gesture_map2[i][2]) {
            false_counter++;
        }
    }
    if (false_counter>2) {
        return false;
    } else {
        return true;
    }
}