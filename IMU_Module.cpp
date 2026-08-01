#include "IMU_Module.h"
#include "Globals.h"
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <MahonyAHRS.h>

// --- TinyML Includes ---
#include "seismic_model.h"
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>

// MPU6050 can be 0x68 or 0x69 depending on AD0 pin.
#define MPU6050_ADDR_PRIMARY 0x69
#define MPU6050_ADDR_FALLBACK 0x68

// Model input: 200 samples * 5 features = 1000 floats.
#define NUMBER_OF_INPUTS 1000
#define NUMBER_OF_OUTPUTS 4
#define FEATURES_PER_SAMPLE 5
#define SAMPLES_PER_WINDOW 200

// Keep this bigger because the model contains SHAPE and CNN layers.
#define TENSOR_ARENA_SIZE (48 * 1024)
#define TF_NUM_OPS 16

// Timing
#define IMU_SAMPLE_DELAY_MS 10
#define INFERENCE_INTERVAL_MS 250
#define SERIAL_DEBUG_INTERVAL_MS 1000

// Rule-based detection tuning.
// NOTE: these are empirically-chosen starting points from bench testing on
// one unit, not a calibrated-per-device or dataset-derived set of numbers.
// If you see too many false TREMOR/LANDSLIDE triggers (or missed events),
// set LOG_CALIBRATION_METRICS to 1 below, collect a few minutes of CSV
// output across IDLE / TREMOR / LANDSLIDE / SEISMIC motion, and re-derive
// these thresholds from that data rather than adjusting them blind.
#define TREMOR_LINEAR_ACC_THRESHOLD 0.08f
#define TREMOR_GYRO_THRESHOLD 18.0f
#define SEISMIC_LINEAR_ACC_THRESHOLD 0.28f
#define SEISMIC_GYRO_THRESHOLD 90.0f
#define LANDSLIDE_TILT_THRESHOLD 28.0f
#define LANDSLIDE_LINEAR_ACC_THRESHOLD 0.05f
#define EVENT_HOLD_MS 1200

// --- Concurrency ---
// i2cMutex protects the shared I2C bus. Every previous xSemaphoreTake() call
// used portMAX_DELAY, i.e. an unbounded wait -- if any task holding the
// mutex ever hangs, everything else on the bus deadlocks forever with no
// way to recover or even detect it. All takes now use a bounded timeout;
// on timeout we skip that cycle instead of blocking indefinitely.
#define I2C_MUTEX_TIMEOUT_TICKS pdMS_TO_TICKS(50)

// detectionMux protects the fields written here on core 1 (imuTask) and
// read from core 0 (loop(), via the getters below). `volatile` alone does
// NOT make a multi-byte read/write atomic across cores -- this closes that
// gap with a real (cheap, uncontended) critical section.
static portMUX_TYPE detectionMux = portMUX_INITIALIZER_UNLOCKED;

// --- Feature normalization (see README "Contribution Notes") ---
// The README's retraining guidance says normalization constants must be
// baked into firmware so inference input matches training input. Previously
// nothing here actually normalized anything, which is a real train/inference
// mismatch risk if the shipped model *was* trained on normalized data.
// These are placeholders -- compute real per-feature mean/std from your
// training set and fill them in. USE_FEATURE_NORMALIZATION defaults to 0
// so this patch does not silently change the shipped model's behavior;
// flip it to 1 only after you've filled in real constants below.
#define USE_FEATURE_NORMALIZATION 0
// Order matches the feature order used everywhere else: roll, pitch, ax, ay, az
static const float FEATURE_MEAN[FEATURES_PER_SAMPLE] = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static const float FEATURE_STD[FEATURES_PER_SAMPLE]  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// Set to 1 to print raw motion metrics as CSV every sample, for offline
// threshold calibration. Leave at 0 for normal operation -- this is verbose.
#define LOG_CALIBRATION_METRICS 0

static Eloquent::TF::Sequential<TF_NUM_OPS, TENSOR_ARENA_SIZE> ml;

// True circular buffer for incoming samples -- O(1) per-sample write.
// The flat `ringBuffer` the model actually consumes is assembled
// (linearized) only right before inference, at most every
// INFERENCE_INTERVAL_MS -- not on every 10ms sample. The previous
// implementation did a ~4KB memmove on every single sample (~400KB/s of
// memory traffic) just to keep a sliding window; this does the same job
// for a fraction of the cost.
static float sampleRing[SAMPLES_PER_WINDOW][FEATURES_PER_SAMPLE];
static float ringBuffer[NUMBER_OF_INPUTS] = {0.0f}; // linearized inference input
static int ringWriteIdx = 0;

static Mahony filter;

static uint8_t mpuAddress = MPU6050_ADDR_PRIMARY;
static bool imuReady = false;
static bool aiReady = false;

static float currentRoll = 0.0f;
static float currentPitch = 0.0f;
static float currentAccX = 0.0f;
static float currentAccY = 0.0f;
static float currentAccZ = 0.0f;
static float currentGyroX = 0.0f;
static float currentGyroY = 0.0f;
static float currentGyroZ = 0.0f;

static float rollOffset = 0.0f;
static float pitchOffset = 0.0f;

static volatile int predictedClass = 0;
static volatile float predictionConfidence = 0.0f;
static volatile float motionScore = 0.0f;
static volatile float linearAcceleration = 0.0f;
static volatile float gyroMagnitude = 0.0f;
static volatile float tiltMagnitude = 0.0f;
static volatile int detectionSource = 0;

static float gyroX_bias = 0.0f;
static float gyroY_bias = 0.0f;
static float gyroZ_bias = 0.0f;

static int sampleCount = 0;
static uint32_t lastInferenceMs = 0;
static uint32_t lastDebugMs = 0;
static uint32_t eventHoldUntilMs = 0;
static int heldClass = 0;
static float heldConfidence = 0.0f;
static int heldSource = 0;

static bool i2cWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

static bool i2cReadBytes(uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  size_t received = Wire.requestFrom((uint16_t)mpuAddress, length, true);
  if (received < length) {
    return false;
  }
  for (size_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

static bool pingMPU(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static bool detectMPUAddress() {
  if (pingMPU(MPU6050_ADDR_PRIMARY)) {
    mpuAddress = MPU6050_ADDR_PRIMARY;
    return true;
  }
  if (pingMPU(MPU6050_ADDR_FALLBACK)) {
    mpuAddress = MPU6050_ADDR_FALLBACK;
    return true;
  }
  return false;
}

// O(1): write the newest sample into the circular buffer and advance the
// write pointer. No shifting of existing data.
static void pushToBuffer(float r, float p, float ax, float ay, float az) {
  sampleRing[ringWriteIdx][0] = r;
  sampleRing[ringWriteIdx][1] = p;
  sampleRing[ringWriteIdx][2] = ax;
  sampleRing[ringWriteIdx][3] = ay;
  sampleRing[ringWriteIdx][4] = az;
  ringWriteIdx = (ringWriteIdx + 1) % SAMPLES_PER_WINDOW;

  if (sampleCount < SAMPLES_PER_WINDOW) {
    sampleCount++;
  }
}

// Flattens the circular buffer into chronological (oldest-to-newest) order
// for the model, applying feature normalization if enabled. Only called
// right before inference (see runDetection()), i.e. at most every
// INFERENCE_INTERVAL_MS -- this is the one place the O(SAMPLES_PER_WINDOW)
// cost is actually paid, instead of on every sample.
static void linearizeWindowForInference() {
  // Called only once the buffer is full (guarded in runDetection), so the
  // oldest sample is always exactly at the current write pointer.
  int oldest = ringWriteIdx;
  for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
    int src = (oldest + i) % SAMPLES_PER_WINDOW;
    int dst = i * FEATURES_PER_SAMPLE;
    for (int f = 0; f < FEATURES_PER_SAMPLE; f++) {
      float v = sampleRing[src][f];
#if USE_FEATURE_NORMALIZATION
      v = (v - FEATURE_MEAN[f]) / FEATURE_STD[f];
#endif
      ringBuffer[dst + f] = v;
    }
  }
}

static void registerModelOps() {
  ml.resolver.AddConv2D();
  ml.resolver.AddMaxPool2D();
  ml.resolver.AddFullyConnected();
  ml.resolver.AddReshape();
  ml.resolver.AddShape();
  ml.resolver.AddRelu();
  ml.resolver.AddSoftmax();
  // These are common in Keras-converted TFLite models.
  // If your library does not compile one of these, comment that one line only.
  ml.resolver.AddPad();
  ml.resolver.AddMean();
  ml.resolver.AddStridedSlice();
  ml.resolver.AddPack();
  ml.resolver.AddConcatenation();
}

static int argmaxOutput() {
  int best = 0;
  float bestValue = ml.outputs[0];
  for (int i = 1; i < NUMBER_OF_OUTPUTS; i++) {
    if (ml.outputs[i] > bestValue) {
      bestValue = ml.outputs[i];
      best = i;
    }
  }
  return best;
}

static float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static void computeMotionMetrics(float gx, float gy, float gz, float ax, float ay, float az, float roll, float pitch) {
  float accMag = sqrtf(ax * ax + ay * ay + az * az);
  float linAcc = fabsf(accMag - 1.0f);
  float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  float tiltMag = fabsf(roll) + fabsf(pitch);

  // Smooth values so the display is stable, but still responsive.
  // These fields are read from the other core via getters, so writes are
  // done inside a critical section (see detectionMux above).
  float newLinAcc, newGyroMag, newTiltMag;
  portENTER_CRITICAL(&detectionMux);
  newLinAcc  = (linearAcceleration * 0.75f) + (linAcc * 0.25f);
  newGyroMag = (gyroMagnitude * 0.75f) + (gyroMag * 0.25f);
  newTiltMag = (tiltMagnitude * 0.80f) + (tiltMag * 0.20f);
  linearAcceleration = newLinAcc;
  gyroMagnitude = newGyroMag;
  tiltMagnitude = newTiltMag;
  portEXIT_CRITICAL(&detectionMux);

  float accScore = newLinAcc / SEISMIC_LINEAR_ACC_THRESHOLD;
  float gyroScore = newGyroMag / SEISMIC_GYRO_THRESHOLD;
  float tiltScore = newTiltMag / LANDSLIDE_TILT_THRESHOLD;

  float score = accScore;
  if (gyroScore > score) score = gyroScore;
  if (tiltScore > score) score = tiltScore;

  portENTER_CRITICAL(&detectionMux);
  motionScore = clamp01(score);
  portEXIT_CRITICAL(&detectionMux);

#if LOG_CALIBRATION_METRICS
  // CSV: millis,linearAcc,gyroMag,tiltMag,motionScore -- pipe this to a file
  // and use it to re-derive the thresholds above from real data.
  Serial.print(millis()); Serial.print(',');
  Serial.print(newLinAcc, 4); Serial.print(',');
  Serial.print(newGyroMag, 2); Serial.print(',');
  Serial.print(newTiltMag, 2); Serial.print(',');
  Serial.println(clamp01(score), 4);
#endif
}

static void ruleBasedDetection(int &ruleClass, float &ruleConfidence, float linAcc, float gyroMag, float tiltMag, float mScore) {
  ruleClass = 0;
  ruleConfidence = 0.0f;

  bool strongShake = (linAcc >= SEISMIC_LINEAR_ACC_THRESHOLD) ||
                      (gyroMag >= SEISMIC_GYRO_THRESHOLD);

  bool tremor = (linAcc >= TREMOR_LINEAR_ACC_THRESHOLD) ||
                (gyroMag >= TREMOR_GYRO_THRESHOLD);

  bool landslide = (tiltMag >= LANDSLIDE_TILT_THRESHOLD &&
                     linAcc >= LANDSLIDE_LINEAR_ACC_THRESHOLD);

  if (strongShake) {
    ruleClass = 3; // SEISMIC
    ruleConfidence = clamp01(0.65f + mScore * 0.35f);
    return;
  }
  if (landslide) {
    ruleClass = 2; // LANDSLIDE
    ruleConfidence = clamp01(0.60f + mScore * 0.35f);
    return;
  }
  if (tremor) {
    ruleClass = 1; // TREMOR
    ruleConfidence = clamp01(0.50f + mScore * 0.35f);
    return;
  }
  ruleClass = 0;
  ruleConfidence = clamp01(1.0f - mScore);
}

static void printDebugLine(int modelClass, float modelConfidence, int ruleClass, float ruleConfidence,
                            float linAcc, float gyroMag, float tiltMag, float mScore,
                            int finalClassSnapshot, float finalConfSnapshot, int finalSourceSnapshot) {
  uint32_t now = millis();
  if (now - lastDebugMs < SERIAL_DEBUG_INTERVAL_MS) {
    return;
  }
  lastDebugMs = now;

  Serial.println();
  Serial.println("===== IMU + AI DEBUG =====");
  Serial.print("MPU addr: 0x"); Serial.println(mpuAddress, HEX);
  Serial.print("Samples: "); Serial.print(sampleCount); Serial.print("/"); Serial.println(SAMPLES_PER_WINDOW);
  Serial.print("Roll/Pitch: "); Serial.print(currentRoll, 2); Serial.print(" / "); Serial.println(currentPitch, 2);
  Serial.print("Acc(g): "); Serial.print(currentAccX, 3); Serial.print(", "); Serial.print(currentAccY, 3); Serial.print(", "); Serial.println(currentAccZ, 3);
  Serial.print("Gyro(dps): "); Serial.print(currentGyroX, 1); Serial.print(", "); Serial.print(currentGyroY, 1); Serial.print(", "); Serial.println(currentGyroZ, 1);
  Serial.print("LinearAcc: "); Serial.print(linAcc, 3);
  Serial.print(" | GyroMag: "); Serial.print(gyroMag, 1);
  Serial.print(" | TiltMag: "); Serial.print(tiltMag, 1);
  Serial.print(" | MotionScore: "); Serial.println(mScore, 3);

  if (aiReady) {
    Serial.print("ML OUT[0] IDLE: "); Serial.println(ml.outputs[0], 6);
    Serial.print("ML OUT[1] TREMOR: "); Serial.println(ml.outputs[1], 6);
    Serial.print("ML OUT[2] LANDSLIDE: "); Serial.println(ml.outputs[2], 6);
    Serial.print("ML OUT[3] SEISMIC: "); Serial.println(ml.outputs[3], 6);
    Serial.print("ML class/conf: "); Serial.print(modelClass); Serial.print(" / "); Serial.println(modelConfidence, 4);
  } else {
    Serial.println("ML: disabled / not loaded");
  }

  Serial.print("Rule class/conf: "); Serial.print(ruleClass); Serial.print(" / "); Serial.println(ruleConfidence, 4);
  Serial.print("FINAL class/conf/source: "); Serial.print(finalClassSnapshot); Serial.print(" / "); Serial.print(finalConfSnapshot, 4); Serial.print(" / "); Serial.println(finalSourceSnapshot);
  Serial.println("==========================");
}

static void runDetection() {
  // Snapshot the smoothed metrics once under the lock, then work with
  // plain locals for the rest of this function -- avoids holding the
  // critical section for the whole (much longer) detection logic below.
  float linAcc, gyroMag, tiltMag, mScore;
  portENTER_CRITICAL(&detectionMux);
  linAcc = linearAcceleration;
  gyroMag = gyroMagnitude;
  tiltMag = tiltMagnitude;
  mScore = motionScore;
  portEXIT_CRITICAL(&detectionMux);

  int ruleClass = 0;
  float ruleConfidence = 0.0f;
  ruleBasedDetection(ruleClass, ruleConfidence, linAcc, gyroMag, tiltMag, mScore);

  int modelClass = 0;
  float modelConfidence = 0.0f;
  bool modelValid = false;

  uint32_t now = millis();
  if (aiReady && sampleCount >= SAMPLES_PER_WINDOW && now - lastInferenceMs >= INFERENCE_INTERVAL_MS) {
    lastInferenceMs = now;
    linearizeWindowForInference(); // O(SAMPLES_PER_WINDOW), but only here, every INFERENCE_INTERVAL_MS
    if (ml.predict(ringBuffer).isOk()) {
      modelClass = argmaxOutput();
      modelConfidence = ml.outputs[modelClass];
      modelValid = true;
    } else {
      Serial.print(">>> Inference Error: ");
      Serial.println(ml.exception.toString());
    }
  }

  // Decision logic:
  // 1. If physical motion is detected but ML says IDLE, override ML.
  // 2. If ML confidently detects a non-idle class, accept ML.
  // 3. Otherwise use rule detector.
  int finalClass = ruleClass;
  float finalConfidence = ruleConfidence;
  int finalSource = (sampleCount < SAMPLES_PER_WINDOW) ? 0 : 2;

  if (modelValid && modelClass != 0 && modelConfidence >= 0.50f) {
    finalClass = modelClass;
    finalConfidence = modelConfidence;
    finalSource = 1;
  }

  if (ruleClass != 0 && mScore >= 0.20f) {
    if (!modelValid || modelClass == 0 || modelConfidence < ruleConfidence) {
      finalClass = ruleClass;
      finalConfidence = ruleConfidence;
      finalSource = modelValid ? 3 : 2;
    }
  }

  // Hold events briefly so the OLED/serial does not flicker back to IDLE instantly.
  if (finalClass != 0) {
    heldClass = finalClass;
    heldConfidence = finalConfidence;
    heldSource = finalSource;
    eventHoldUntilMs = now + EVENT_HOLD_MS;
  } else if (now < eventHoldUntilMs) {
    finalClass = heldClass;
    finalConfidence = heldConfidence;
    finalSource = heldSource;
  }

  portENTER_CRITICAL(&detectionMux);
  predictedClass = finalClass;
  predictionConfidence = clamp01(finalConfidence);
  detectionSource = finalSource;
  portEXIT_CRITICAL(&detectionMux);

  printDebugLine(modelClass, modelConfidence, ruleClass, ruleConfidence,
                 linAcc, gyroMag, tiltMag, mScore,
                 finalClass, clamp01(finalConfidence), finalSource);
}

static void imuTask(void *pvParameters) {
  uint32_t lastMicros = micros();

  for (;;) {
    uint32_t currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;
    if (dt <= 0.0f || dt > 0.5f) {
      dt = 0.01f;
    }

    bool success = false;
    uint8_t raw[14];

    if (imuReady && i2cMutex != NULL) {
      if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT_TICKS)) {
        success = i2cReadBytes(0x3B, raw, 14);
        xSemaphoreGive(i2cMutex);
      }
      // else: bus was busy this cycle -- skip and try again next tick
      // instead of blocking the whole task indefinitely.
    }

    if (success) {
      int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
      int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
      int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);
      int16_t raw_gx = (int16_t)((raw[8] << 8) | raw[9]);
      int16_t raw_gy = (int16_t)((raw[10] << 8) | raw[11]);
      int16_t raw_gz = (int16_t)((raw[12] << 8) | raw[13]);

      float gx_deg = (raw_gx - gyroX_bias) / 131.0f;
      float gy_deg = (raw_gy - gyroY_bias) / 131.0f;
      float gz_deg = (raw_gz - gyroZ_bias) / 131.0f;

      float ax_g = raw_ax / 16384.0f;
      float ay_g = raw_ay / 16384.0f;
      float az_g = raw_az / 16384.0f;

      filter.updateIMU(gx_deg, gy_deg, gz_deg, ax_g, ay_g, az_g);

      float roll = filter.getRoll();
      float pitch = filter.getPitch();

      // These are read from core 0 via getRoll()/getPitch()/getAccX() etc.,
      // so writes go through the same critical section as the detection
      // state above.
      portENTER_CRITICAL(&detectionMux);
      currentRoll = roll;
      currentPitch = pitch;
      currentAccX = ax_g;
      currentAccY = ay_g;
      currentAccZ = az_g;
      currentGyroX = gx_deg;
      currentGyroY = gy_deg;
      currentGyroZ = gz_deg;
      portEXIT_CRITICAL(&detectionMux);

      computeMotionMetrics(gx_deg, gy_deg, gz_deg, ax_g, ay_g, az_g, roll, pitch);

      // O(1) now -- see pushToBuffer() above.
      pushToBuffer(roll, pitch, ax_g, ay_g, az_g);
      runDetection();
    }

    vTaskDelay(IMU_SAMPLE_DELAY_MS / portTICK_PERIOD_MS);
  }
}

void initIMU() {
  Serial.println(">>> Initializing IMU + detection engine...");

  ml.setNumInputs(NUMBER_OF_INPUTS);
  ml.setNumOutputs(NUMBER_OF_OUTPUTS);
  registerModelOps();

  if (!ml.begin(seismic_model).isOk()) {
    aiReady = false;
    Serial.print(">>> TinyML model failed. Error: ");
    Serial.println(ml.exception.toString());
    Serial.println(">>> Rule-based detector will still work.");
  } else {
    aiReady = true;
    Serial.println(">>> TinyML model loaded successfully.");
  }

  filter.begin(100);

  if (i2cMutex == NULL) {
    Serial.println(">>> IMU failed: i2cMutex is NULL.");
    imuReady = false;
    return;
  }

  if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT_TICKS)) {
    imuReady = detectMPUAddress();
    if (!imuReady) {
      xSemaphoreGive(i2cMutex);
      Serial.println(">>> MPU6050 not found at 0x69 or 0x68.");
      return;
    }

    Serial.print(">>> MPU6050 found at 0x");
    Serial.println(mpuAddress, HEX);

    i2cWriteRegister(0x6B, 0x00); // Wake up
    delay(100);
    i2cWriteRegister(0x1A, 0x03); // DLPF
    i2cWriteRegister(0x1B, 0x00); // Gyro ±250 dps
    i2cWriteRegister(0x1C, 0x00); // Accel ±2g

    Serial.println(">>> Calibrating gyro. Keep device still...");
    const int samples = 500;
    gyroX_bias = 0.0f;
    gyroY_bias = 0.0f;
    gyroZ_bias = 0.0f;

    for (int i = 0; i < samples; i++) {
      uint8_t graw[6];
      if (i2cReadBytes(0x43, graw, 6)) {
        int16_t gx = (int16_t)((graw[0] << 8) | graw[1]);
        int16_t gy = (int16_t)((graw[2] << 8) | graw[3]);
        int16_t gz = (int16_t)((graw[4] << 8) | graw[5]);
        gyroX_bias += gx;
        gyroY_bias += gy;
        gyroZ_bias += gz;
      }
      delay(3);
    }

    gyroX_bias /= (float)samples;
    gyroY_bias /= (float)samples;
    gyroZ_bias /= (float)samples;

    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println(">>> IMU init failed: could not acquire I2C mutex.");
    imuReady = false;
    return;
  }

  Serial.println(">>> Gyro calibration complete.");
  Serial.print("GX bias: "); Serial.println(gyroX_bias);
  Serial.print("GY bias: "); Serial.println(gyroY_bias);
  Serial.print("GZ bias: "); Serial.println(gyroZ_bias);

  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    imuTask,
    "IMU_Task",
    10000,
    NULL,
    2,
    NULL,
    1
  );

  if (taskCreated == pdPASS) {
    Serial.println(">>> IMU task started.");
  } else {
    Serial.println(">>> Failed to start IMU task.");
  }
}

void tareIMU() {
  portENTER_CRITICAL(&detectionMux);
  rollOffset = currentRoll;
  pitchOffset = currentPitch;
  portEXIT_CRITICAL(&detectionMux);
  Serial.println(">>> IMU tare complete.");
}

float getRoll() {
  portENTER_CRITICAL(&detectionMux);
  float v = currentRoll - rollOffset;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getPitch() {
  portENTER_CRITICAL(&detectionMux);
  float v = currentPitch - pitchOffset;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getAccX() {
  portENTER_CRITICAL(&detectionMux);
  float v = currentAccX;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getAccY() {
  portENTER_CRITICAL(&detectionMux);
  float v = currentAccY;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getAccZ() {
  portENTER_CRITICAL(&detectionMux);
  float v = currentAccZ;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

int getSeismicClass() {
  portENTER_CRITICAL(&detectionMux);
  int v = predictedClass;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getSeismicConfidence() {
  portENTER_CRITICAL(&detectionMux);
  float v = predictionConfidence;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getMotionScore() {
  portENTER_CRITICAL(&detectionMux);
  float v = motionScore;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getLinearAcceleration() {
  portENTER_CRITICAL(&detectionMux);
  float v = linearAcceleration;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getGyroMagnitude() {
  portENTER_CRITICAL(&detectionMux);
  float v = gyroMagnitude;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

float getTiltMagnitude() {
  portENTER_CRITICAL(&detectionMux);
  float v = tiltMagnitude;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}

bool isAIModelReady() {
  return aiReady;
}

int getDetectionSource() {
  portENTER_CRITICAL(&detectionMux);
  int v = detectionSource;
  portEXIT_CRITICAL(&detectionMux);
  return v;
}
