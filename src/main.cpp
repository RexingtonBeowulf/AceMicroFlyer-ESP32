/*
  ╔══════════════════════════════════════════════════════════════╗
  ║   ESP32-C3 Drone — PID (fixed D-on-measurement + LPF)       ║
  ║                                                              ║
  ║  BLE write commands:                                         ║
  ║    Flight:   "T:<0-100>,P:<deg>,R:<deg>,Y:<-100-100>"        ║
  ║    PID tune: "PID:R,<kP>,<kI>,<kD>"   (Roll)                ║
  ║              "PID:P,<kP>,<kI>,<kD>"   (Pitch)               ║
  ║              "PID:Y,<kP>,<kI>,<kD>"   (Yaw)                 ║
  ║    Motor test: "TEST:<0-4>,<0-100>"                          ║
  ║    Step capture: "CAPTURE"  → arm 2s buffer, then stream     ║
  ║    IMU/Gyro: "IMU"  "GYRO"  "GYROSTOP"                      ║
  ║                                                              ║
  ║  Motor pins (corrected): 0=FL(5) 1=FR(4) 2=BL(2) 3=BR(3)   ║
  ╚══════════════════════════════════════════════════════════════╝
*/

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// ── Motor pins (corrected assignment) ──────────────────────
const uint8_t MOTOR_PINS[4] = {5, 4, 2, 3};  // FL FR BL BR
const char*   MOTOR_NAMES[4] = {"FL(GPIO5)","FR(GPIO4)","BL(GPIO2)","BR(GPIO3)"};

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;
const int DUTY_MIN = 0;
const int DUTY_MAX = 1023;

// ── MPU-6050 ────────────────────────────────────────────────
const uint8_t MPU_ADDR   = 0x68;
const float   ACCEL_SENS = 16384.0f;
const float   GYRO_SENS  = 131.0f;

float gyroX_off=0, gyroY_off=0, gyroZ_off=0;
float accX_off=0,  accY_off=0,  accZ_off=0;

// ── Complementary filter ────────────────────────────────────
const float CF_ALPHA = 0.98f;
float cf_roll=0, cf_pitch=0;
float gz_rate=0;   // yaw rate °/s

// ── PID gains (live-adjustable) ─────────────────────────────
struct PIDGains { float kP, kI, kD; };
PIDGains rollGains  = {1.2f, 0.004f, 0.08f};
PIDGains pitchGains = {1.2f, 0.004f, 0.08f};
PIDGains yawGains   = {2.0f, 0.010f, 0.00f};

const float I_CLAMP = 80.0f;
const float D_LPF_ALPHA = 0.8f;  // D-term LPF (0=no filter, 1=heavy)
struct PIDState { float integral=0, prevError=0, prevMeasured=0, filteredDeriv=0; };
PIDState rollState, pitchState, yawState;

// ── Setpoints ───────────────────────────────────────────────
volatile int   sp_throttle  = 0;
volatile float sp_roll      = 0;
volatile float sp_pitch     = 0;
volatile float sp_yaw_rate  = 0;
const float    MAX_YAW_RATE = 90.0f;

// ── Safety ──────────────────────────────────────────────────
const unsigned long WATCHDOG_MS = 500;
unsigned long lastPacketMs = 0;
bool motorsArmed = false;
bool testMode    = false;
bool gyroLive    = false;

// ═══════════════════════════════════════════════════════════
//  Ring buffer for step response logging
//  Each record: timestamp(ms), setpoint, measured, roll_corr, pitch_corr
// ═══════════════════════════════════════════════════════════
const int  LOG_SIZE = 600;   // 6 seconds at 100 Hz
struct LogRecord {
  uint16_t dt_ms;     // ms since capture start (fits in uint16 for 65 s)
  int8_t   setpoint;  // degrees × 1 (fits ±127°)
  int8_t   measured;  // degrees × 1
  uint16_t motorFL;
  uint16_t motorFR;
};

LogRecord logBuf[LOG_SIZE];
int       logHead    = 0;
int       logCount   = 0;
bool      logArmed   = false;   // true = currently logging
bool      logPending = false;   // true = log full, waiting to stream
unsigned long logStartMs = 0;
int8_t    logAxis    = 0;       // 0=roll, 1=pitch

// Non-blocking log streaming
bool logStreaming = false;
int logStreamIndex = 0;
unsigned long lastStreamMs = 0;

// Which setpoint/measured to record (set by CAPTURE command)
volatile float* log_setpoint_ptr = &sp_roll;
volatile float* log_measured_ptr = &cf_roll;

// ── BLE ─────────────────────────────────────────────────────
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHAR_UUID    "abcdefab-cdef-abcd-efab-cdefabcdefab"
#define NOTIFY_UUID  "abcdefab-cdef-abcd-efab-cdefabcdef00"

BLEServer*         pServer    = nullptr;
BLECharacteristic* pCharWrite = nullptr;
BLECharacteristic* pCharNotify= nullptr;
bool bleConnected = false;

// ═══════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════
int   clamp_i(int v,   int lo,   int hi)   { return v<lo?lo:(v>hi?hi:v); }
float clamp_f(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }

void stopAllMotors() { for(int i=0;i<4;i++) ledcWrite(i,0); }

void bleNotify(const char* msg) {
  if(!bleConnected||!pCharNotify) return;
  pCharNotify->setValue((uint8_t*)msg, strlen(msg));
  pCharNotify->notify();
}

// ═══════════════════════════════════════════════════════════
//  MPU-6050
// ═══════════════════════════════════════════════════════════
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
  Wire.requestFrom(MPU_ADDR,(uint8_t)14,true);
  RawIMU r;
  r.ax=Wire.read()<<8|Wire.read();
  r.ay=Wire.read()<<8|Wire.read();
  r.az=Wire.read()<<8|Wire.read();
  Wire.read(); Wire.read();
  r.gx=Wire.read()<<8|Wire.read();
  r.gy=Wire.read()<<8|Wire.read();
  r.gz=Wire.read()<<8|Wire.read();
  return r;
}

void calibrateIMU() {
  Serial.println("[CAL] Keep drone FLAT & STILL for 3 s...");
  delay(2000);
  const int N=500;
  long sAx=0,sAy=0,sAz=0,sGx=0,sGy=0,sGz=0;
  for(int i=0;i<N;i++){
    RawIMU r=readRawIMU();
    sAx+=r.ax; sAy+=r.ay; sAz+=r.az;
    sGx+=r.gx; sGy+=r.gy; sGz+=r.gz;
    delay(4);
  }
  accX_off=sAx/(float)N;
  accY_off=sAy/(float)N;
  accZ_off=sAz/(float)N - ACCEL_SENS;
  gyroX_off=sGx/(float)N;
  gyroY_off=sGy/(float)N;
  gyroZ_off=sGz/(float)N;
  Serial.printf("[CAL] Gyro offsets: %.1f %.1f %.1f\n",
    gyroX_off,gyroY_off,gyroZ_off);
}

// ═══════════════════════════════════════════════════════════
//  PID
// ═══════════════════════════════════════════════════════════
float computePID(PIDState& s, const PIDGains& g,
                 float setpoint, float measured, float dt) {
  float err = setpoint - measured;
  s.integral = clamp_f(s.integral + err*dt, -I_CLAMP, I_CLAMP);

  // Derivative on MEASUREMENT (not error) — avoids kick on setpoint change
  float rawDeriv = -(measured - s.prevMeasured) / dt;
  s.prevMeasured = measured;

  // Low-pass filter on D term to tame gyro noise
  s.filteredDeriv = D_LPF_ALPHA * s.filteredDeriv + (1.f - D_LPF_ALPHA) * rawDeriv;

  return g.kP*err + g.kI*s.integral + g.kD*s.filteredDeriv;
}

void resetPID() {
  rollState={}; pitchState={}; yawState={};
}

// ═══════════════════════════════════════════════════════════
//  Motor mixer
// ═══════════════════════════════════════════════════════════
int lastFL=0,lastFR=0,lastBL=0,lastBR=0;

// applyMix: rollCorr from rollPID, pitchCorr from pitchPID, yawCorr from yawPID
void applyMix(int throttle, float rollCorr, float pitchCorr, float yawCorr) {
  if(throttle<3){ stopAllMotors(); return; }
  int base = map(throttle,0,100,DUTY_MIN,DUTY_MAX);
  lastFR = clamp_i((int)(base-pitchCorr+rollCorr+yawCorr), DUTY_MIN, DUTY_MAX);
  lastFL = clamp_i((int)(base-pitchCorr-rollCorr-yawCorr), DUTY_MIN, DUTY_MAX);
  lastBR = clamp_i((int)(base+pitchCorr+rollCorr-yawCorr), DUTY_MIN, DUTY_MAX);
  lastBL = clamp_i((int)(base+pitchCorr-rollCorr+yawCorr), DUTY_MIN, DUTY_MAX);
  ledcWrite(0,lastFL); ledcWrite(1,lastFR);
  ledcWrite(2,lastBL); ledcWrite(3,lastBR);
}

// ═══════════════════════════════════════════════════════════
//  Ring buffer logging
// ═══════════════════════════════════════════════════════════
void logSample() {
  if(!logArmed) return;
  uint32_t elapsed = millis() - logStartMs;
  LogRecord rec;
  rec.dt_ms    = (uint16_t)min(elapsed, (uint32_t)65535);
  rec.setpoint = (int8_t)clamp_f((float)*log_setpoint_ptr, -127, 127);
  rec.measured = (int8_t)clamp_f(*log_measured_ptr, -127, 127);
  rec.motorFL  = (uint16_t)lastFL;
  rec.motorFR  = (uint16_t)lastFR;

  logBuf[logHead] = rec;
  logHead = (logHead + 1) % LOG_SIZE;
  if(logCount < LOG_SIZE) logCount++;

  if(elapsed >= 5000) {   // 5 second capture window
    logArmed   = false;
    logPending = true;
    Serial.printf("[LOG] Capture complete: %d samples\n", logCount);
    bleNotify("LOG:READY");
  }
}

// Stream the log over BLE as CSV text chunks
// Format per chunk: "D:<idx>,<dt>,<sp>,<meas>,<fl>,<fr>\n" repeated
void streamLog() {
    if (!bleConnected || logCount == 0) return;

    Serial.println("[LOG] Starting non-blocking stream of " + String(logCount) + " samples");
    bleNotify("LOG:START");
    delay(30);                     // small safe delay after START

    logStreaming = true;
    logStreamIndex = 0;
    lastStreamMs = millis();
}

// ═══════════════════════════════════════════════════════════
//  Packet parser
// ═══════════════════════════════════════════════════════════
void parsePacket(const std::string& s) {
  lastPacketMs = millis();

  // ── PID tune: "PID:R,1.5,0.005,0.09" ──────────────────
  if(s.rfind("PID:",0)==0) {
    char axis = s[4];
    float kp,ki,kd;
    if(sscanf(s.c_str()+6, "%f,%f,%f", &kp,&ki,&kd)==3) {
      PIDGains* g = nullptr;
      PIDState* st = nullptr;
      if(axis=='R'||axis=='r'){ g=&rollGains;  st=&rollState; }
      if(axis=='P'||axis=='p'){ g=&pitchGains; st=&pitchState; }
      if(axis=='Y'||axis=='y'){ g=&yawGains;   st=&yawState; }
      if(g) {
        g->kP=kp; g->kI=ki; g->kD=kd;
        *st = {};  // reset integrator when gains change
        Serial.printf("[PID] %c → kP=%.4f kI=%.5f kD=%.4f\n",axis,kp,ki,kd);
        char buf[48];
        snprintf(buf,48,"PID:ACK:%c,%.3f,%.4f,%.3f",axis,kp,ki,kd);
        bleNotify(buf);
      }
    }
    return;
  }

  // ── Step capture: "CAPTURE:R" or "CAPTURE:P" ───────────
  if(s.rfind("CAPTURE",0)==0) {
    char axis = (s.size()>8) ? s[8] : 'R';
    logAxis = (axis=='P'||axis=='p') ? 1 : 0;
    log_setpoint_ptr = (logAxis==1) ? &sp_pitch : &sp_roll;
    log_measured_ptr = (logAxis==1) ? &cf_pitch : &cf_roll;
    logHead    = 0;
    logCount   = 0;
    logPending = false;
    logStartMs = millis();
    logArmed   = true;
    motorsArmed= true;
    Serial.printf("[LOG] Armed on %s axis\n", logAxis?"Pitch":"Roll");
    bleNotify("LOG:ARMED");
    return;
  }

  // ── Stream captured data ────────────────────────────────
  if(s=="LOGSTREAM") {
    streamLog();
    return;
  }

  // ── Motor test ──────────────────────────────────────────
  if(s.rfind("TEST:",0)==0) {
    int motor,duty;
    if(sscanf(s.c_str(),"TEST:%d,%d",&motor,&duty)==2) {
      testMode = true;
      motor = clamp_i(motor,0,4);
      duty  = clamp_i(duty,0,100);
      motorsArmed = (duty>0);
      stopAllMotors();
      if(duty>0) {
        int d = (int)(duty/100.0f*DUTY_MAX);
        if(motor==4) { for(int i=0;i<4;i++) ledcWrite(i,d); }
        else ledcWrite(motor, d);
        Serial.printf("[TEST] Motor %d → %d%%\n",motor,duty);
      } else {
        testMode=false;
      }
      char buf[32]; snprintf(buf,32,"TEST:ACK:%d,%d%%",motor,duty);
      bleNotify(buf);
    }
    return;
  }

  // ── IMU snapshot ────────────────────────────────────────
  if(s=="IMU") {
    char buf[48];
    snprintf(buf,48,"IMU:R=%.1f,P=%.1f",cf_roll,cf_pitch);
    Serial.println(buf);
    bleNotify(buf);
    return;
  }

  // ── Gyro live ───────────────────────────────────────────
  if(s=="GYRO")     { gyroLive=true;  bleNotify("GYRO:ON");  return; }
  if(s=="GYROSTOP") { gyroLive=false; bleNotify("GYRO:OFF"); return; }

  // ── Normal flight ────────────────────────────────────────
  testMode = false;
  int tI,pI,rI,yI;
  if(sscanf(s.c_str(),"T:%d,P:%d,R:%d,Y:%d",&tI,&pI,&rI,&yI)>=1) {
    sp_throttle = clamp_i(tI,0,100);
    sp_pitch    = clamp_f((float)pI,-30.f,30.f);
    sp_roll     = clamp_f((float)rI,-30.f,30.f);
    sp_yaw_rate = clamp_f((float)yI/100.f*MAX_YAW_RATE,-MAX_YAW_RATE,MAX_YAW_RATE);
    motorsArmed = true;
  }
}

// ═══════════════════════════════════════════════════════════
//  BLE callbacks
// ═══════════════════════════════════════════════════════════
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected=true;
    Serial.println("[BLE] Connected");
    // Send current gains on connect so app can sync
    char buf[80];
    snprintf(buf,80,"PID:ACK:R,%.3f,%.4f,%.3f",
      rollGains.kP,rollGains.kI,rollGains.kD);
    delay(200); bleNotify(buf);
  }
  void onDisconnect(BLEServer*) override {
    bleConnected=false;
    motorsArmed=false; testMode=false; gyroLive=false; logArmed=false;
    stopAllMotors(); resetPID();
    Serial.println("[BLE] Disconnected");
    BLEDevice::startAdvertising();
  }
};

class CharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    parsePacket(c->getValue());
  }
};

// ═══════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println("\n=== ESP32-C3 Drone — Live PID Tuning + Data Logging ===");

  for(int i=0;i<4;i++){
    ledcSetup(i,PWM_FREQ,PWM_RES);
    ledcAttachPin(MOTOR_PINS[i],i);
    ledcWrite(i,0);
  }

  Wire.begin(6,7); Wire.setClock(400000);
  mpuWriteReg(0x6B,0x00); delay(100);
  mpuWriteReg(0x1B,0x00);
  mpuWriteReg(0x1C,0x00);
  mpuWriteReg(0x1A,0x03);
  delay(50);

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75);
  Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR,1,true);
  uint8_t who=Wire.read();
  Serial.printf("[IMU] WHO_AM_I=0x%02X %s\n",who,who==0x68?"OK":"WARNING");

  calibrateIMU();

  // Seed filter
  RawIMU r=readRawIMU();
  float ax=(r.ax-accX_off)/ACCEL_SENS;
  float ay=(r.ay-accY_off)/ACCEL_SENS;
  float az=(r.az-accZ_off)/ACCEL_SENS;
  cf_roll  = atan2f(ay,az)*RAD_TO_DEG;
  cf_pitch = atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;

  BLEDevice::init("ESP32-C3-Drone");
  pServer=BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService* svc=pServer->createService(BLEUUID(SERVICE_UUID),30);
  pCharWrite=svc->createCharacteristic(CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_WRITE_NR);
  pCharWrite->setCallbacks(new CharCB());
  pCharWrite->addDescriptor(new BLE2902());

  pCharNotify=svc->createCharacteristic(NOTIFY_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharNotify->addDescriptor(new BLE2902());

  svc->start();
  BLEAdvertising* adv=BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising as 'ESP32-C3-Drone'");
}

// ═══════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════
unsigned long lastFastUs=0, lastSlowUs=0;
unsigned long lastPrintMs=0, lastGyroMs=0;

void loop() {
  unsigned long now = micros();

  // ── 500 Hz: IMU + complementary filter ─────────────────
  if(now-lastFastUs >= 2000) {
    float dt=(now-lastFastUs)*1e-6f;
    lastFastUs=now;

    RawIMU r=readRawIMU();
    float ax=(r.ax-accX_off)/ACCEL_SENS;
    float ay=(r.ay-accY_off)/ACCEL_SENS;
    float az=(r.az-accZ_off)/ACCEL_SENS;
    float gx=(r.gx-gyroX_off)/GYRO_SENS;
    float gy=(r.gy-gyroY_off)/GYRO_SENS;
    float gz=(r.gz-gyroZ_off)/GYRO_SENS;
    gz_rate=gz;

    float acc_roll  = atan2f(ay,az)*RAD_TO_DEG;
    float acc_pitch = atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;
    cf_roll  = CF_ALPHA*(cf_roll  + gx*dt) + (1.f-CF_ALPHA)*acc_roll;
    cf_pitch = CF_ALPHA*(cf_pitch + gy*dt) + (1.f-CF_ALPHA)*acc_pitch;

    // Gyro live stream
    if(gyroLive && millis()-lastGyroMs>100) {
      lastGyroMs=millis();
      char buf[48];
      snprintf(buf,48,"GYRO:R=%.1f,P=%.1f",cf_roll,cf_pitch);
      bleNotify(buf);
    }

    // ── 100 Hz: PID + motors + logging ───────────────────
    if(now-lastSlowUs >= 10000) {
      float pidDt=(now-lastSlowUs)*1e-6f;
      lastSlowUs=now;

      // Watchdog
      if(motorsArmed && !testMode && !logArmed &&
         (millis()-lastPacketMs > WATCHDOG_MS)) {
        stopAllMotors(); motorsArmed=false; resetPID();
      }

      if(!testMode && bleConnected && motorsArmed) {
        float rC=computePID(rollState, rollGains, sp_roll,     cf_roll,  pidDt);
        float pC=computePID(pitchState,pitchGains,sp_pitch,    cf_pitch, pidDt);
        float yC=computePID(yawState,  yawGains,  sp_yaw_rate, gz_rate,  pidDt);
        applyMix(sp_throttle, rC, pC, yC);
        logSample();  // only writes when logArmed
      }

      // Serial debug every 500 ms
      if(millis()-lastPrintMs>500) {
        lastPrintMs=millis();
        Serial.printf("R=%+5.1f° P=%+5.1f° | T=%d | kP_r=%.3f kI_r=%.4f kD_r=%.3f\n",
          cf_roll,cf_pitch,sp_throttle,
          rollGains.kP,rollGains.kI,rollGains.kD);
      }

          // ── Non-blocking log streaming (~50 notifies/sec max) ─────────────────
      if (logStreaming && bleConnected) {
        if (millis() - lastStreamMs >= 20) {        // ← 20 ms = ~50 Hz, safe for NimBLE
            lastStreamMs = millis();

            int start = (logCount < LOG_SIZE) ? 0 : logHead;
            int idx = (start + logStreamIndex) % LOG_SIZE;
            LogRecord& r = logBuf[idx];

            char buf[64];
            snprintf(buf, sizeof(buf), "D:%d,%u,%d,%d,%u,%u\n",
                     logStreamIndex, r.dt_ms, r.setpoint, r.measured,
                     r.motorFL, r.motorFR);

            pCharNotify->setValue((uint8_t*)buf, strlen(buf));
            pCharNotify->notify();

            logStreamIndex++;

            if (logStreamIndex >= logCount) {
                bleNotify("LOG:END");
                Serial.printf("[LOG] Stream complete: sent %d samples\n", logCount);
                logStreaming = false;
                logPending = false;
                logCount = 0;
                logHead = 0;
            }
        }
      }
    }
  }
}