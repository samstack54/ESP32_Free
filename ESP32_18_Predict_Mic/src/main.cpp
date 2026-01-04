#include <Arduino.h>

#define EI_DEBUG 0
#define EI_CLASSIFIER_DEBUG 0
#define EI_PORTING_ARDUINO_DEBUG 0

#include <ESP32S3_Free_Audio_inferencing.h>
#include "driver/i2s.h"

// ---------------- Pins (your wiring) ----------------
static const int I2S_BCLK = 12; // SCK
static const int I2S_WS   = 11; // WS
static const int I2S_DIN  = 10; // SD (mic -> ESP32)
static const int LED_PIN  = 2;

// Touch on GPIO1 = T1
static const int TOUCH_PIN = T1;
static const uint32_t TOUCH_THRESHOLD = 50000;
static const uint32_t TOUCH_RELEASE   = 40000;
static const uint32_t START_DELAY_MS  = 200;

// ---------------- Audio / EI settings ----------------
#define SAMPLE_RATE     16000U
#define SAMPLE_BITS     32 // I2S read as 32-bit words from INMP441

// EI window: 1000ms => 16000 samples. Increment 500ms => 8000 samples.
static const uint32_t WINDOW_SAMPLES = EI_CLASSIFIER_RAW_SAMPLE_COUNT; // should be 16000
static const uint32_t HOP_SAMPLES    = (SAMPLE_RATE / 2);              // 8000

// Tune these
static const int AUDIO_SHIFT = 14;   // start 13; 14 if too loud, 12 if too quiet
static const int AUDIO_GAIN  = 1;    // start 2~3. If clipping, reduce.

// Run multiple overlapping windows after touch
static const int NUM_WINDOWS = 1;

// Energy gate: if (max-min) below this, treat as silence and skip
static const int SILENCE_RANGE_THRESH = 2500;

// DMA chunk
static const uint32_t CHUNK_FRAMES = 256;
static int32_t i2sRawBuffer[CHUNK_FRAMES];

// ---------------- Inference buffer ----------------
static int16_t windowBuf[WINDOW_SAMPLES];

// ---------------- State machine ----------------
enum RunState { WAIT_TOUCH, WAIT_2S, RUN_KWS, WAIT_RELEASE };
static RunState state = WAIT_TOUCH;
static uint32_t t0 = 0;

// ---------------- Helpers ----------------
static bool i2s_init_inmp441();
static bool fill_samples(int16_t *dst, uint32_t n_samples);
static bool read_and_append(int16_t *dst, uint32_t old_keep, uint32_t new_read);
static void compute_minmax(const int16_t *buf, uint32_t n, int16_t &mn, int16_t &mx);
static int  microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (!i2s_init_inmp441()) {
    Serial.println("ERROR: I2S init failed");
    while (1) delay(100);
  }

  Serial.println("KWS Touch Trigger Ready");
  Serial.println("ARMED");
}

void loop() {
  uint32_t tv = touchRead(TOUCH_PIN);

  switch (state) {
    case WAIT_TOUCH:
      if (tv > TOUCH_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);
        t0 = millis();
        state = WAIT_2S;
      }
      break;

    case WAIT_2S:
      if (millis() - t0 >= START_DELAY_MS) {
        state = RUN_KWS;
      }
      break;

    case RUN_KWS: {
      // 1) Fill first 1-second window (16000 samples)
      if (!fill_samples(windowBuf, WINDOW_SAMPLES)) {
        Serial.println("ERR: fill_samples failed");
        digitalWrite(LED_PIN, LOW);
        Serial.println("DONE");
        state = WAIT_RELEASE;
        break;
      }

      // Prepare EI signal
      signal_t signal;
      signal.total_length = WINDOW_SAMPLES;
      signal.get_data = &microphone_audio_signal_get_data;

      // Track best non-silent across windows
      int best_label = -1;
      float best_score = -1.0f;
      String best_name = "silent";

      for (int w = 0; w < NUM_WINDOWS; w++) {
        // Energy gate
        int16_t mn, mx;
        compute_minmax(windowBuf, WINDOW_SAMPLES, mn, mx);
        int range = (int)mx - (int)mn;
        Serial.printf("window min=%d max=%d range=%d\n", mn, mx, range);

        if (range >= SILENCE_RANGE_THRESH) {
          ei_impulse_result_t result = {0};
          EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);
          if (r != EI_IMPULSE_OK) {
            Serial.printf("ERR: run_classifier (%d)\n", r);
          } else {
            // Find best label for this window
            int bi = 0;
            float bv = 0.f;
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
              float v = result.classification[ix].value;
              if (v > bv) { bv = v; bi = (int)ix; }
            }

            const char *label = result.classification[bi].label;
            Serial.print("BEST: ");
            Serial.print(label);
            Serial.print(" ");
            Serial.println(bv, 3);

            // Prefer non-silent labels, but still track best score
            if (strcmp(label, "silent") != 0) {
              if (bv > best_score) {
                best_score = bv;
                best_label = bi;
                best_name = label;
              }
            } else {
              // If we never get non-silent, we will fall back later
              if (best_label < 0 && bv > best_score) {
                best_score = bv;
                best_name = label;
              }
            }
          }
        } else {
          Serial.println("SKIP (silence window)");
        }

        // 2) Slide by 500ms except after last window
        if (w < NUM_WINDOWS - 1) {
          // Keep last 8000 samples, append 8000 new samples
          // windowBuf = [old(16000)] => shift left by 8000, fill last 8000
          memmove(&windowBuf[0], &windowBuf[HOP_SAMPLES],
                  (WINDOW_SAMPLES - HOP_SAMPLES) * sizeof(int16_t));

          if (!fill_samples(&windowBuf[WINDOW_SAMPLES - HOP_SAMPLES], HOP_SAMPLES)) {
            Serial.println("ERR: hop fill failed");
            break;
          }
        }
      }

      // Final decision print
      // Serial.print("FINAL: ");
      // Serial.print(best_name);
      // Serial.print(" ");
      // Serial.println(best_score, 3);

      digitalWrite(LED_PIN, LOW);
      Serial.println("DONE");
      state = WAIT_RELEASE;
    } break;

    case WAIT_RELEASE:
      if (tv < TOUCH_RELEASE) {
        Serial.println("ARMED");
        Serial.println(" ");
        state = WAIT_TOUCH;
      }
      break;
  }

  delay(5);
}

// ---------------- ESP-IDF I2S init ----------------
static bool i2s_init_inmp441() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,      // INMP441 L/R -> GND = Left
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_DIN
  };

  esp_err_t e;
  e = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (e != ESP_OK) return false;

  e = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (e != ESP_OK) return false;

  i2s_zero_dma_buffer(I2S_NUM_0);
  return true;
}

// ---------------- Read N samples into dst ----------------
static bool fill_samples(int16_t *dst, uint32_t n_samples) {
  uint32_t written = 0;

  while (written < n_samples) {
    size_t bytes_read = 0;
    const size_t bytes_to_read = CHUNK_FRAMES * sizeof(int32_t);

    esp_err_t e = i2s_read(I2S_NUM_0, (void*)i2sRawBuffer,
                           bytes_to_read, &bytes_read, portMAX_DELAY);
    if (e != ESP_OK) return false;
    if (bytes_read == 0) continue;

    int n = (int)(bytes_read / sizeof(int32_t));
    if (n <= 0) continue;
    if (n > (int)CHUNK_FRAMES) n = (int)CHUNK_FRAMES;

    // Convert + DC remove per chunk (simple)
    int64_t sum = 0;
    for (int i = 0; i < n; i++) {
      int16_t v = (int16_t)(i2sRawBuffer[i] >> AUDIO_SHIFT);
      sum += v;
    }
    int16_t mean = (int16_t)(sum / n);

    for (int i = 0; i < n && written < n_samples; i++) {
      int32_t v = (int32_t)((int16_t)(i2sRawBuffer[i] >> AUDIO_SHIFT)) - mean;
      v *= AUDIO_GAIN;
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      dst[written++] = (int16_t)v;
    }
  }

  return true;
}

static void compute_minmax(const int16_t *buf, uint32_t n, int16_t &mn, int16_t &mx) {
  mn = 32767;
  mx = -32768;
  for (uint32_t i = 0; i < n; i++) {
    int16_t v = buf[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
}

// EI signal callback reads from windowBuf
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&windowBuf[offset], out_ptr, length);
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Model sensor type must be microphone."
#endif
