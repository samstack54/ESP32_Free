// ESP32_15 Pysical IMU

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

const int PIN_SDA = 7;
const int PIN_SCL = 6;

const int TOUCH_PIN = T1;      // GPIO1 touch channel
const int TOUCH_TH  = 50000;
const int LED_PIN = 2;

// Recording timing
const uint32_t ARM_DELAY_MS  = 2000;
const uint32_t DURATION_MS   = 10000;
const uint32_t INTERVAL_MS   = 16;     // ~62.5 Hz

float ACCEL_SCALE = 16384.0f;  // default for ±2g

MPU6050 mpu;

enum State { IDLE, ARMED_WAIT, RECORDING };
State state = IDLE;

uint32_t armStartMs    = 0;
uint32_t recordStartMs = 0;
uint32_t nextSampleMs  = 0;

uint32_t lastTouchMs = 0;
const uint32_t TOUCH_DEBOUNCE_MS = 300;

bool isTouched() {
  int v = touchRead(TOUCH_PIN);
  return (v > TOUCH_TH);
}

void startRecording() {
  digitalWrite(LED_PIN, HIGH);

  recordStartMs = millis();
  nextSampleMs  = recordStartMs;

  Serial.println("START");
  Serial.println("time_ms,ax,ay,az");
  state = RECORDING;
}

void stopRecording() {
  digitalWrite(LED_PIN, LOW);
  Serial.println("DONE");
  state = IDLE;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(300);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  mpu.initialize();

  // Set accel range AFTER initialize (this is the key)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  ACCEL_SCALE = 2048.0f;  // ±16g => 2048 LSB/g

  Serial.println("MPU6050 ready (Accel range = ±16g).");
  Serial.println("Touch GPIO1 (T1) to arm. Starts 2s later, then records 10s.");
}

void loop() {
  uint32_t now = millis();

  if (state == IDLE) {
    if (isTouched() && (now - lastTouchMs > TOUCH_DEBOUNCE_MS)) {
      lastTouchMs = now;
      armStartMs = now;
      state = ARMED_WAIT;
      Serial.println("ARMED");
    }
    return;
  }

  if (state == ARMED_WAIT) {
    if (now - armStartMs >= ARM_DELAY_MS) startRecording();
    return;
  }

  if (state == RECORDING) {
    if (now - recordStartMs >= DURATION_MS) {
      stopRecording();
      delay(200);
      return;
    }

    if ((int32_t)(now - nextSampleMs) >= 0) {
      int16_t rawAx, rawAy, rawAz;
      mpu.getAcceleration(&rawAx, &rawAy, &rawAz);

      float ax = rawAx / ACCEL_SCALE;
      float ay = rawAy / ACCEL_SCALE;
      float az = rawAz / ACCEL_SCALE;

      uint32_t t_ms = now - recordStartMs;

      Serial.print(t_ms); Serial.print(',');
      Serial.print(ax, 5); Serial.print(',');
      Serial.print(ay, 5); Serial.print(',');
      Serial.println(az, 5);

      nextSampleMs += INTERVAL_MS;
      if ((int32_t)(now - nextSampleMs) > (int32_t)(4 * INTERVAL_MS)) {
        nextSampleMs = now + INTERVAL_MS;
      }
    }
  }
}
