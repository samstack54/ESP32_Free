#include <Arduino.h>

#include <ESP32_Free_IMU_inferencing.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

#define CONVERT_G_TO_MS2 9.80665f
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

void setup() {
    Serial.begin(115200);
    Wire.begin(7,6);
    mpu.initialize();

    pinMode(2, OUTPUT);

    if (!mpu.testConnection()) {
        ei_printf("MPU6050 connection failed\n");
        while (1);
    }

    ei_printf("MPU6050 ready\n");
}

void loop() {

    Serial.print("Ready to START   touchRead = ");
    Serial.println(touchRead(T1));

    while(touchRead(T1) < 50000)
    { delay(10);}
    digitalWrite(2, HIGH);

    for (int i = 0; i < EI_CLASSIFIER_RAW_SAMPLE_COUNT; i++) {
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);
        features[i * 3 + 0] = ax / 16384.0f * CONVERT_G_TO_MS2;
        features[i * 3 + 1] = ay / 16384.0f * CONVERT_G_TO_MS2;
        features[i * 3 + 2] = az / 16384.0f * CONVERT_G_TO_MS2;
        delay((int)(1000.0f / EI_CLASSIFIER_FREQUENCY));
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
        memcpy(out_ptr, features + offset, length * sizeof(float));
        return 0;
    };

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

    if (res != EI_IMPULSE_OK) {
        ei_printf("run_classifier failed (%d)\n", res);
        return;
    }

    ei_printf("Predictions:\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("  %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
    
    // Find the label with the highest confidence
    const char* top_label = result.classification[0].label;
    float top_score = result.classification[0].value;

    for (size_t ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > top_score) {
            top_score = result.classification[ix].value;
            top_label = result.classification[ix].label;
        }
    }
    ei_printf("Predicted: %s (%.5f)\n\n", top_label, top_score);
    digitalWrite(2, LOW);

}
