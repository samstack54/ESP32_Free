#include <Arduino.h>
#include <ESP32S3_Free_image_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

#include <edge-impulse-sdk/classifier/ei_run_classifier.h>
#include <edge-impulse-sdk/porting/ei_classifier_porting.h>

// Freenove ESP32-S3 WROOM camera pins
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y2_GPIO_NUM      11
#define Y3_GPIO_NUM      9
#define Y4_GPIO_NUM      8
#define Y5_GPIO_NUM      10
#define Y6_GPIO_NUM      12
#define Y7_GPIO_NUM      18
#define Y8_GPIO_NUM      17
#define Y9_GPIO_NUM      16
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

// Raw camera frame size (QVGA)
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// Tune these
static float DETECTION_THRESHOLD = 0.60f; // try 0.50, 0.60, 0.70, 0.80
static int INFERENCE_DELAY_MS = 200;      // reduce spam / heat

static bool debug_nn = false;
static bool is_initialised = false;
static uint8_t *snapshot_buf = nullptr;

// Camera config
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("Edge Impulse Inferencing Demo");

    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    } else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("Model input: %dx%d\n", (int)EI_CLASSIFIER_INPUT_WIDTH, (int)EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("Detection threshold: %.2f\n", DETECTION_THRESHOLD);

    ei_printf("\nStarting continuous inference in 2 seconds...\n");
    ei_sleep(2000);
}

void loop() {
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS *
                                    EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
                                    EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        ei_sleep(1000);
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((uint32_t)EI_CLASSIFIER_INPUT_WIDTH,
                          (uint32_t)EI_CLASSIFIER_INPUT_HEIGHT,
                          snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        snapshot_buf = nullptr;
        ei_sleep(INFERENCE_DELAY_MS);
        return;
    }

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        snapshot_buf = nullptr;
        ei_sleep(INFERENCE_DELAY_MS);
        return;
    }

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    // Find best box + print only boxes above threshold
    float best_v = 0.0f;
    ei_impulse_result_bounding_box_t best_bb = {0};
    bool any = false;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        auto bb = result.bounding_boxes[i];

        if (bb.value > best_v) {
            best_v = bb.value;
            best_bb = bb;
        }

        if (bb.value < DETECTION_THRESHOLD) continue;
        any = true;

        ei_printf("  %s (%.3f) [x:%u y:%u w:%u h:%u]\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }

    if (!any) {
        ei_printf("  No objects >= %.2f (best=%s %.3f)\n",
                  DETECTION_THRESHOLD,
                  best_bb.label ? best_bb.label : "none",
                  best_v);
    } else {
        ei_printf("  BEST: %s (%.3f)\n", best_bb.label, best_v);
    }
#else
    ei_printf("Predictions:\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: %.5f\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\n", result.anomaly);
#endif

    free(snapshot_buf);
    snapshot_buf = nullptr;

    ei_sleep(INFERENCE_DELAY_MS);
}

bool ei_camera_init(void) {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    // Optional: tune sensor parameters a bit (can help reduce false positives)
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // Try these if needed:
        // s->set_framesize(s, FRAMESIZE_QVGA);
        // s->set_brightness(s, 1);
        // s->set_contrast(s, 1);
        // s->set_saturation(s, 0);
        // s->set_gainceiling(s, (gainceiling_t)2);
    }

    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) ||
        (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {

        int r = ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            (int)img_width,
            (int)img_height
        );

        if (r != 0) {
            ei_printf("crop_and_interpolate_rgb888 failed (%d)\n", r);
            return false;
        }
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] =
            (snapshot_buf[pixel_ix + 2] << 16) +
            (snapshot_buf[pixel_ix + 1] << 8) +
            snapshot_buf[pixel_ix];

        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif