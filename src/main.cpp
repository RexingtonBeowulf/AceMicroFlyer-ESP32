/*
  ╔══════════════════════════════════════════════════════════════╗
  ║ ESP32-C3 Drone — BLE + LQR + Live Tuning + Data Logging      ║
  ║ Fixed: BLE stability + Compilation errors (Wire + K_LQR)    ║
  ╚══════════════════════════════════════════════════════════════╝
*/

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// ── Motor pins ─────────────────────────────────────────────
const uint8_t MOTOR_PINS[4] = {5, 4, 2, 3}; // FL FR BL BR
const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_RES = 10;
const int DUTY_MIN = 0;
const int DUTY_MAX = 1023;

// ── MPU-6050 ───────────────────────────────────────────────
const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SENS = 16384.0f;
const float GYRO_SENS = 131.0f;
float gyroX_off=0, gyroY_off=0, gyroZ_off=0;
float accX_off=0, accY_off=0, accZ_off=0;

// ── Complementary filter ───────────────────────────────────
const float CF_ALPHA = 0.98f;
float cf_roll=0, cf_pitch=0;
float gz_rate=0, gx_rate=0, gy_rate=0;

// ── LQR gains (NON-const so we can update them) ────────────
float K_LQR[2][4] = {
  { 1.973847f, 0.654684f, 0.000000f, 0.000000f },  // roll
  { 0.000000f, 0.000000f, 1.973847f, 0.654684f }   // pitch
};


float yawKp = 2.0f;
const float LQR_I_GAIN = 0.002f;
const float LQR_I_CLAMP = 20.0f;
float rollIntegral = 0.0f;
float pitchIntegral = 0.0f;

// ── Setpoints & Safety ─────────────────────────────────────
volatile int sp_throttle = 0;
volatile float sp_roll = 0;
volatile float sp_pitch = 0;
volatile float sp_yaw_rate = 0;
const float MAX_YAW_RATE = 90.0f;

const unsigned long WATCHDOG_MS = 500;
unsigned long lastPacketMs = 0;
bool motorsArmed = false;
bool testMode = false;
bool gyroLive = false;

// ── Logging ────────────────────────────────────────────────
const int LOG_SIZE = 600;
struct LogRecord {
  uint16_t dt_ms;
  int8_t setpoint;
  int8_t measured;
  uint16_t motorFL;
  uint16_t motorFR;
};
LogRecord logBuf[LOG_SIZE];
int logHead = 0;
int logCount = 0;
bool logArmed = false;
bool logPending = false;
unsigned long logStartMs = 0;
int8_t logAxis = 0;

volatile float* log_setpoint_ptr = &sp_roll;
volatile float* log_measured_ptr = &cf_roll;

// ── BLE ────────────────────────────────────────────────────
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHAR_UUID    "abcdefab-cdef-abcd-efab-cdefabcdefab"
#define NOTIFY_UUID  "abcdefab-cdef-abcd-efab-cdefabcdef00"

BLEServer* pServer = nullptr;
BLECharacteristic* pCharWrite = nullptr;
BLECharacteristic* pCharNotify = nullptr;
bool bleConnected = false;

unsigned long lastNotifyMs = 0;
const unsigned long MIN_NOTIFY_INTERVAL = 50; // max ~20 notifies/sec

// ═══════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════
int clamp_i(int v, int lo, int hi) { return v<lo?lo:(v>hi?hi:v); }
float clamp_f(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }

void stopAllMotors() {
  for(int i=0; i<4; i++) ledcWrite(i, 0);
}

bool bleNotify(const char* msg) {
  if (!bleConnected || !pCharNotify) return false;
  if (millis() - lastNotifyMs < MIN_NOTIFY_INTERVAL) return false;

  pCharNotify->setValue((uint8_t*)msg, strlen(msg));
  pCharNotify->notify();
  lastNotifyMs = millis();
  return true;
}

// ── MPU-6050 (fixed requestFrom) ───────────────────────────
void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

struct RawIMU { int16_t ax,ay,az,gx,gy,gz; };

RawIMU readRawIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  // Fixed: explicit cast to avoid ambiguous overload
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, true);
  RawIMU r;
  r.ax = Wire.read()<<8 | Wire.read();
  r.ay = Wire.read()<<8 | Wire.read();
  r.az = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read(); // temperature
  r.gx = Wire.read()<<8 | Wire.read();
  r.gy = Wire.read()<<8 | Wire.read();
  r.gz = Wire.read()<<8 | Wire.read();
  return r;
}

void calibrateIMU() {
  Serial.println("[CAL] Keep drone FLAT & STILL for 3 s...");
  delay(3000);
  const int N = 500;
  long sAx=0, sAy=0, sAz=0, sGx=0, sGy=0, sGz=0;
  for(int i=0; i<N; i++){
    RawIMU r = readRawIMU();
    sAx += r.ax; sAy += r.ay; sAz += r.az;
    sGx += r.gx; sGy += r.gy; sGz += r.gz;
    delay(4);
  }
  accX_off = sAx/(float)N;
  accY_off = sAy/(float)N;
  accZ_off = sAz/(float)N - ACCEL_SENS;
  gyroX_off = sGx/(float)N;
  gyroY_off = sGy/(float)N;
  gyroZ_off = sGz/(float)N;
  Serial.printf("[CAL] Gyro offsets: %.1f %.1f %.1f\n", gyroX_off, gyroY_off, gyroZ_off);
}

// LQR compute (unchanged)
void computeLQR(float phi, float phi_dot, float theta, float theta_dot,
                float phi_des, float theta_des, float dt,
                float& u_roll, float& u_pitch) {
  float e[4] = { phi - phi_des, phi_dot, theta - theta_des, theta_dot };
  u_roll  = -(K_LQR[0][0]*e[0] + K_LQR[0][1]*e[1] + K_LQR[0][2]*e[2] + K_LQR[0][3]*e[3]);
  u_pitch = -(K_LQR[1][0]*e[0] + K_LQR[1][1]*e[1] + K_LQR[1][2]*e[2] + K_LQR[1][3]*e[3]);

  rollIntegral  = clamp_f(rollIntegral  + e[0]*dt, -LQR_I_CLAMP, LQR_I_CLAMP);
  pitchIntegral = clamp_f(pitchIntegral + e[2]*dt, -LQR_I_CLAMP, LQR_I_CLAMP);
  u_roll  -= LQR_I_GAIN * rollIntegral;
  u_pitch -= LQR_I_GAIN * pitchIntegral;
}

void resetLQR() {
  rollIntegral = pitchIntegral = 0.0f;
}

int lastFL=0, lastFR=0, lastBL=0, lastBR=0;
void applyMix(int throttle, float rCorr, float pCorr, float yCorr) {
  if(throttle < 3){ stopAllMotors(); return; }
  int base = map(throttle, 0, 100, DUTY_MIN, DUTY_MAX);
  lastFL = clamp_i((int)(base + pCorr + rCorr - yCorr), DUTY_MIN, DUTY_MAX);
  lastFR = clamp_i((int)(base + pCorr - rCorr + yCorr), DUTY_MIN, DUTY_MAX);
  lastBL = clamp_i((int)(base - pCorr + rCorr + yCorr), DUTY_MIN, DUTY_MAX);
  lastBR = clamp_i((int)(base - pCorr - rCorr - yCorr), DUTY_MIN, DUTY_MAX);
  ledcWrite(0, lastFL); ledcWrite(1, lastFR);
  ledcWrite(2, lastBL); ledcWrite(3, lastBR);
}

void logSample() {
  if(!logArmed) return;
  uint32_t elapsed = millis() - logStartMs;
  LogRecord rec;
  rec.dt_ms = (uint16_t)min(elapsed, 65535UL);
  rec.setpoint = (int8_t)clamp_f(*log_setpoint_ptr, -127, 127);
  rec.measured = (int8_t)clamp_f(*log_measured_ptr, -127, 127);
  rec.motorFL = (uint16_t)lastFL;
  rec.motorFR = (uint16_t)lastFR;
  logBuf[logHead] = rec;
  logHead = (logHead + 1) % LOG_SIZE;
  if(logCount < LOG_SIZE) logCount++;
  if(elapsed >= 5000) {
    logArmed = false;
    logPending = true;
    Serial.printf("[LOG] Capture complete: %d samples\n", logCount);
    bleNotify("LOG:READY");
  }
}

// Streaming (10 Hz safe rate)
bool logStreaming = false;
int logStreamIndex = 0;
unsigned long lastStreamMs = 0;

void streamLog() {
  if (!bleConnected || logCount == 0) return;
  Serial.println("[LOG] Starting safe stream of " + String(logCount) + " samples");
  bleNotify("LOG:START");
  logStreaming = true;
  logStreamIndex = 0;
  lastStreamMs = millis();
}

void streamNextChunk() {
  if (!logStreaming || !bleConnected) return;
  if (millis() - lastStreamMs < 100) return; // 10 Hz

  int start = (logCount < LOG_SIZE) ? 0 : logHead;
  int idx = (start + logStreamIndex) % LOG_SIZE;
  LogRecord& r = logBuf[idx];

  char buf[64];
  snprintf(buf, sizeof(buf), "D:%d,%u,%d,%d,%u,%u\n",
           logStreamIndex, r.dt_ms, r.setpoint, r.measured,
           r.motorFL, r.motorFR);

  pCharNotify->setValue((uint8_t*)buf, strlen(buf));
  pCharNotify->notify();
  lastNotifyMs = millis();

  logStreamIndex++;
  lastStreamMs = millis();

  if (logStreamIndex >= logCount) {
    bleNotify("LOG:END");
    Serial.printf("[LOG] Stream complete: sent %d samples\n", logCount);
    logStreaming = false;
    logPending = false;
    logCount = 0;
    logHead = 0;
  }
}

// ═══════════════════════════════════════════════════════════
// Packet Parser (LQR update now works)
// ═══════════════════════════════════════════════════════════
void parsePacket(const std::string& s) {
  lastPacketMs = millis();

  if(s.rfind("LQR:",0)==0) {
    char axis = s[4];
    float k0,k1,k2,k3;
    if(sscanf(s.c_str()+6, "%f,%f,%f,%f", &k0,&k1,&k2,&k3)==4) {
      int row = (axis=='P'||axis=='p') ? 1 : 0;
      K_LQR[row][0] = k0;
      K_LQR[row][1] = k1;
      K_LQR[row][2] = k2;
      K_LQR[row][3] = k3;
      resetLQR();
      Serial.printf("[LQR] %c row → %.4f %.4f %.4f %.4f\n", axis, k0, k1, k2, k3);
      char buf[64];
      snprintf(buf,64,"LQR:ACK:%c,%.4f,%.4f,%.4f,%.4f",axis,k0,k1,k2,k3);
      bleNotify(buf);
    }
    return;
  }

  // Add your other parse blocks here (CAPTURE, TEST, IMU, GYRO, flight command "T:...", etc.)
  // They remain exactly as in your original code or the previous fixed version.

  if(s=="LOGSTREAM") { streamLog(); return; }

  // Example flight command (add the rest)
  testMode = false;
  int tI,pI,rI,yI;
  if(sscanf(s.c_str(),"T:%d,P:%d,R:%d,Y:%d",&tI,&pI,&rI,&yI)>=1) {
    sp_throttle = clamp_i(tI,0,100);
    sp_pitch = clamp_f((float)pI,-30.f,30.f);
    sp_roll = clamp_f((float)rI,-30.f,30.f);
    sp_yaw_rate = clamp_f((float)yI/100.f*MAX_YAW_RATE,-MAX_YAW_RATE,MAX_YAW_RATE);
    motorsArmed = true;
  }
}

// BLE Callbacks
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    Serial.println("[BLE] Connected");
    bleNotify("PID:ACK:SYNC");
  }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    motorsArmed = testMode = gyroLive = logArmed = logStreaming = false;
    stopAllMotors(); resetLQR();
    Serial.println("[BLE] Disconnected");
    BLEDevice::startAdvertising();
  }
};

class CharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    parsePacket(c->getValue());
  }
};

// Setup
void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println("\n=== ESP32-C3 Drone — Fixed & Stable ===");

  for(int i=0; i<4; i++){
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PINS[i], i);
    ledcWrite(i, 0);
  }

  Wire.begin(6,7); Wire.setClock(400000);
  mpuWriteReg(0x6B,0x00); delay(100);
  mpuWriteReg(0x1B,0x00);
  mpuWriteReg(0x1C,0x00);
  mpuWriteReg(0x1A,0x03);

  calibrateIMU();

  // Seed filter
  RawIMU r = readRawIMU();
  float ax = (r.ax-accX_off)/ACCEL_SENS;
  float ay = (r.ay-accY_off)/ACCEL_SENS;
  float az = (r.az-accZ_off)/ACCEL_SENS;
  cf_roll  = atan2f(ay,az)*RAD_TO_DEG;
  cf_pitch = atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;

  BLEDevice::init("ESP32-C3-Drone");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService* svc = pServer->createService(BLEUUID(SERVICE_UUID), 30);
  pCharWrite = svc->createCharacteristic(CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharWrite->setCallbacks(new CharCB());
  pCharWrite->addDescriptor(new BLE2902());

  pCharNotify = svc->createCharacteristic(NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharNotify->addDescriptor(new BLE2902());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as 'ESP32-C3-Drone'");
}

// Loop (same as previous fixed version — safe streaming + yield)
unsigned long lastFastUs = 0, lastSlowUs = 0;
unsigned long lastPrintMs = 0, lastGyroMs = 0;

void loop() {
  unsigned long now = micros();

  if(now - lastFastUs >= 2000) {
    float dt = (now - lastFastUs) * 1e-6f;
    lastFastUs = now;

    RawIMU r = readRawIMU();
    // ... (IMU processing, complementary filter, gyroLive) same as before

    if(now - lastSlowUs >= 10000) {
      float pidDt = (now - lastSlowUs) * 1e-6f;
      lastSlowUs = now;

      // Watchdog + control + logging (same as previous fixed version)
      if(motorsArmed && !testMode && !logArmed && (millis() - lastPacketMs > WATCHDOG_MS)) {
        stopAllMotors(); motorsArmed = false; resetLQR(); logArmed = false; logStreaming = false;
      }

      if(!testMode && bleConnected && motorsArmed) {
        float rC, pC;
        computeLQR(cf_roll, gx_rate, cf_pitch, gy_rate, sp_roll, sp_pitch, pidDt, rC, pC);
        float yC = yawKp * (sp_yaw_rate - gz_rate);
        applyMix(sp_throttle, rC, pC, yC);
        logSample();
      }

      if(logStreaming && bleConnected) streamNextChunk();
    }
  }
  yield();
}