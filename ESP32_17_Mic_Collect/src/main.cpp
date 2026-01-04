// ESP32-S3 Free send data to Processing (Improved: touch -> wait 2s -> LED on -> START -> stream)
#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_SD   10
#define I2S_WS   11
#define I2S_SCK  12
#define I2S_PORT I2S_NUM_0

#define SAMPLE_RATE     16000
#define RECORD_SECONDS  10
#define BUFFER_LEN      1024

// ---- User controls ----
static const int LED_PIN = 2;

// Touch thresholds (tune if needed)
// Your note: no touch ~30000, touch >50000
static const uint32_t TOUCH_ON_TH  = 50000; // touched when >= this
static const uint32_t TOUCH_OFF_TH = 45000; // released when < this (hysteresis)

static const uint32_t TOUCH_DEBOUNCE_MS = 80;   // must stay touched this long
static const uint32_t ARM_DELAY_MS      = 2000; // wait 2 seconds before start

enum State {
  WAIT_TOUCH,
  DEBOUNCE_TOUCH,
  ARM_DELAY,
  STREAMING,
  WAIT_RELEASE
};

State state = WAIT_TOUCH;
uint32_t t0 = 0;

void setupI2S() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setupI2S();

  Serial.println();
  Serial.println("Ready: Run Processing. Touch T1 to arm. (Will wait 2s then start)");
}

void loop() {
  uint32_t tr = touchRead(T1);

  switch (state) {
    case WAIT_TOUCH:
      if (tr >= TOUCH_ON_TH) {
        t0 = millis();
        state = DEBOUNCE_TOUCH;
      }
      break;

    case DEBOUNCE_TOUCH:
      // If touch drops, cancel
      if (tr < TOUCH_ON_TH) {
        state = WAIT_TOUCH;
        break;
      }
      // If stayed touched long enough, begin 2s arm delay
      if (millis() - t0 >= TOUCH_DEBOUNCE_MS) {
        t0 = millis();
        state = ARM_DELAY;
        Serial.print("TOUCH_OK ");
        Serial.println(tr);
      }
      break;

    case ARM_DELAY:
      if (millis() - t0 >= ARM_DELAY_MS) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("START");            // Processing can use this to begin logging
        t0 = millis();                      // reuse as streaming start time
        state = STREAMING;
      }
      break;

    case STREAMING: {
      if (millis() - t0 >= (uint32_t)RECORD_SECONDS * 1000UL) {
        digitalWrite(LED_PIN, LOW);
        Serial.println("DONE");
        state = WAIT_RELEASE;
        break;
      }

      size_t bytesRead = 0;
      int32_t samples[BUFFER_LEN];

      // Blocking read for a chunk
      i2s_read(I2S_PORT, (void*)samples, sizeof(samples), &bytesRead, portMAX_DELAY);

      int sampleCount = bytesRead / sizeof(int32_t);
      for (int i = 0; i < sampleCount; i++) {
        int16_t val = (int16_t)(samples[i] >> 14); // your scaling
        Serial.println(val);
      }
    } break;

    case WAIT_RELEASE:
      // Prevent immediate retrigger if finger still on pad
      if (tr < TOUCH_OFF_TH) {
        state = WAIT_TOUCH;
        Serial.println("ARMED");
      }
      break;
  }
}
