
// ESP32-S3 Freenove WROOM N16R8 camera img send to Processing

#include <Arduino.h>
#include "esp_camera.h"
#include "base64.h"

// ===== CONFIGURABLE SETTINGS =====
String fileNamePrefix = "";          // File name prefix
bool enableStreaming = true;         // Enable real-time streaming to Processing
int streamingInterval = 50;          // ms
// ==================================

// CORRECT Camera pin configuration for ESP32-S3 Freenove
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

// --------- Forward declarations (THIS is what Arduino IDE did for you) ----------
void initCamera();
void handleSerialCommand(String command);
void streamImage();
void captureAndSaveImage();
void testCamera();
// ------------------------------------------------------------------------------

// Global variables
bool cameraInitialized = false;
bool streamingActive = false;
String currentLabel = "apple";
int imageCount = 0;
unsigned long lastStreamTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nESP32-S3 Camera Ready (PlatformIO)");
  Serial.println("Commands:");
  Serial.println("- start_stream: Start streaming");
  Serial.println("- stop_stream: Stop streaming");
  Serial.println("- capture: Capture and save one image");
  Serial.println("- set_label <label>: Set current label");
  Serial.println("- test_camera: Test camera capture");
  Serial.println("- status: Show system status");
  Serial.println("Ready for commands!");

  initCamera();
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    handleSerialCommand(command);
  }

  // Handle streaming
  if (streamingActive && enableStreaming && cameraInitialized) {
    unsigned long currentTime = millis();
    if (currentTime - lastStreamTime >= (unsigned long)streamingInterval) {
      streamImage();
      lastStreamTime = currentTime;
    }
  }
}

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Conservative settings for reliability
  config.frame_size = FRAMESIZE_QVGA;   // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    cameraInitialized = false;
    return;
  }

  cameraInitialized = true;
  Serial.println("Camera initialized successfully!");
}

void handleSerialCommand(String command) {
  Serial.println("Received: " + command);

  if (command == "start_stream") {
    streamingActive = true;
    Serial.println("STREAM_STARTED");
  }
  else if (command == "stop_stream") {
    streamingActive = false;
    Serial.println("STREAM_STOPPED");
  }
  else if (command == "capture") {
    captureAndSaveImage();
  }
  else if (command.startsWith("set_label ")) {
    currentLabel = command.substring(10);
    Serial.println("LABEL_SET:" + currentLabel);
  }
  else if (command == "test_camera") {
    testCamera();
  }
  else if (command == "status") {
    Serial.println("STATUS:");
    Serial.println("cameraInitialized=" + String(cameraInitialized));
    Serial.println("streamingActive=" + String(streamingActive));
    Serial.println("currentLabel=" + currentLabel);
    Serial.println("imageCount=" + String(imageCount));
  }
  else {
    Serial.println("UNKNOWN_COMMAND:" + command);
  }
}

// Change in main.cpp
void streamImage() {
  if (!cameraInitialized) return;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("STREAM_ERROR:Failed to capture");
    return;
  }

  String base64Image = base64::encode(fb->buf, fb->len);
  // CHANGE "STREAM_FRAME:" to "STREAM_IMAGE:"
  Serial.println("STREAM_IMAGE:" + String(fb->len) + "," + base64Image); 

  esp_camera_fb_return(fb);
}

void captureAndSaveImage() {
  if (!cameraInitialized) {
    Serial.println("CAPTURE_ERROR:Camera not initialized");
    return;
  }

  Serial.println("Capturing image...");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("CAPTURE_ERROR:Failed");
    return;
  }

  String filename = "/";
  if (fileNamePrefix.length() > 0) filename += fileNamePrefix + "_";
  filename += currentLabel + "_" + String(imageCount) + ".jpg";

  String base64Image = base64::encode(fb->buf, fb->len);
  Serial.println("IMAGE_SAVED:" + filename + "," + String(fb->len) + "," + base64Image);

  esp_camera_fb_return(fb);
  imageCount++;
}

void testCamera() {
  Serial.println("Testing camera...");

  if (!cameraInitialized) {
    Serial.println("TEST_RESULT:Camera not initialized");
    initCamera();
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("TEST_RESULT:Capture failed");
    return;
  }

  Serial.printf("TEST_RESULT:Success - %dx%d, %d bytes\n",
                fb->width, fb->height, fb->len);
  esp_camera_fb_return(fb);
}

