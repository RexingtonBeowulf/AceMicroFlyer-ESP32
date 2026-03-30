/*
  ╔══════════════════════════════════════════════════════════════╗
  ║   ESP32-C3 Drone — BLE + PID + Motor Test Mode              ║
  ║                                                              ║
  ║  Normal packets:  "T:<0-100>,P:<deg>,R:<deg>,Y:<-100-100>"  ║
  ║  Test packets:    "TEST:<motor>,<duty>"                      ║
  ║                   motor = 0-3 (FL/FR/BL/BR) or 4 = all      ║
  ║                   duty  = 0-100 (% of DUTY_MAX)             ║
  ║  IMU query:       "IMU"  → serial prints angles             ║
  ║  Gyro orient:     "GYRO" → serial prints raw axes live       ║
  ╚══════════════════════════════════════════════════════════════╝
*/

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// ── Motor pins: index 0=FL 1=FR 2=BL 3=BR ──────────────────
const uint8_t MOTOR_PINS[4] = {5, 4, 2, 3};
const char*   MOTOR_NAMES[4] = {"FL(GPIO2)", "FR(GPIO3)", "BL(GPIO4)", "BR(GPIO5)"};

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;
const int DUTY_MIN = 0;
const int DUTY_MAX = 800;

// ── MPU-6050 ────────────────────────────────────────────────
const uint8_t MPU_ADDR   = 0x68;
const float   ACCEL_SENS = 16384.0f;
const float   GYRO_SENS  = 131.0f;

float gyroX_offset=0, gyroY_offset=0, gyroZ_offset=0;
float accX_offset=0,  accY_offset=0,  accZ_offset=0;

// ── Complementary filter ────────────────────────────────────
const float CF_ALPHA = 0.98f;
float cf_roll=0, cf_pitch=0;

// ── PID ─────────────────────────────────────────────────────
struct PIDGains { float kP, kI, kD; };
PIDGains rollGains  = {5.0f, 0.004f, 0.08f};
PIDGains pitchGains = {5.0f, 0.004f, 0.08f};
PIDGains yawGains   = {2.0f, 0.01f,  0.0f};

const float I_CLAMP = 100.0f;
struct PIDState { float integral=0, prevError=0; };
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

// ── Test mode ───────────────────────────────────────────────
bool     testMode      = false;   // disables PID when true
int      testMotor     = -1;      // -1=none, 0-3=specific, 4=all
int      testDuty      = 0;       // 0-100%

// ── Gyro live mode (serial print loop) ──────────────────────
bool gyroLiveMode = false;

// ── BLE ─────────────────────────────────────────────────────
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHAR_UUID    "abcdefab-cdef-abcd-efab-cdefabcdefab"
#define NOTIFY_UUID  "abcdefab-cdef-abcd-efab-cdefabcdef00"

BLEServer*         pServer       = nullptr;
BLECharacteristic* pCharWrite    = nullptr;
BLECharacteristic* pCharNotify   = nullptr;
bool bleConnected = false;

// ═══════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════
int clamp_i(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }
float clamp_f(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

void stopAllMotors(){
  for(int i=0;i<4;i++) ledcWrite(i,0);
}

void setMotorDutyPct(int ch, int pct){
  // pct = 0-100
  int duty = (int)(clamp_i(pct,0,100) / 100.0f * DUTY_MAX);
  ledcWrite(clamp_i(ch,0,3), duty);
}

// ═══════════════════════════════════════════════════════════
//  MPU helpers
// ═══════════════════════════════════════════════════════════
void mpuWriteReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

struct RawIMU { int16_t ax,ay,az,gx,gy,gz; };

RawIMU readRawIMU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);
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

void calibrateIMU(){
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
  accX_offset=sAx/(float)N;
  accY_offset=sAy/(float)N;
  accZ_offset=sAz/(float)N - ACCEL_SENS;
  gyroX_offset=sGx/(float)N;
  gyroY_offset=sGy/(float)N;
  gyroZ_offset=sGz/(float)N;
  Serial.printf("[CAL] Done. Gyro offsets: %.1f %.1f %.1f\n",
    gyroX_offset,gyroY_offset,gyroZ_offset);
}

// ═══════════════════════════════════════════════════════════
//  PID
// ═══════════════════════════════════════════════════════════
float computePID(PIDState& s, const PIDGains& g,
                 float setpoint, float measured, float dt){
  float err=setpoint-measured;
  s.integral=clamp_f(s.integral+err*dt,-I_CLAMP,I_CLAMP);
  float deriv=(err-s.prevError)/dt;
  s.prevError=err;
  return g.kP*err + g.kI*s.integral + g.kD*deriv;
}

// ═══════════════════════════════════════════════════════════
//  Motor mixer
// ═══════════════════════════════════════════════════════════
void applyMix(int throttle, float rCorr, float pCorr, float yCorr){
  int base=map(throttle,0,100,DUTY_MIN,DUTY_MAX);
  int fl=base+pCorr+rCorr-yCorr;
  int fr=base+pCorr-rCorr+yCorr;
  int bl=base-pCorr+rCorr+yCorr;
  int br=base-pCorr-rCorr-yCorr;
  if(throttle<3){ stopAllMotors(); return; }
  ledcWrite(0,clamp_i(fl,DUTY_MIN,DUTY_MAX));
  ledcWrite(1,clamp_i(fr,DUTY_MIN,DUTY_MAX));
  ledcWrite(2,clamp_i(bl,DUTY_MIN,DUTY_MAX));
  ledcWrite(3,clamp_i(br,DUTY_MIN,DUTY_MAX));
}

// ═══════════════════════════════════════════════════════════
//  BLE notify helper — sends a short status string back
// ═══════════════════════════════════════════════════════════
void bleNotify(const char* msg){
  if(!bleConnected || !pCharNotify) return;
  pCharNotify->setValue((uint8_t*)msg, strlen(msg));
  pCharNotify->notify();
}

// ═══════════════════════════════════════════════════════════
//  Packet parser
//  Handles:
//    Normal:    "T:50,P:10,R:-5,Y:0"
//    Test:      "TEST:0,30"     → motor 0 at 30%
//               "TEST:4,0"      → all motors off
//    IMU dump:  "IMU"
//    Gyro live: "GYRO"
//    Gyro stop: "GYROSTOP"
// ═══════════════════════════════════════════════════════════
void parsePacket(const std::string& s){
  lastPacketMs = millis();

  if(s.rfind("TEST:",0)==0){
    // ── Test mode ──────────────────────────────────────────
    int motor=-1, duty=0;
    if(sscanf(s.c_str(),"TEST:%d,%d",&motor,&duty)==2){
      testMode  = true;
      testMotor = clamp_i(motor,0,4);
      testDuty  = clamp_i(duty,0,100);
      motorsArmed = (duty>0);

      // Apply immediately
      stopAllMotors();
      if(duty>0){
        if(testMotor==4){
          for(int i=0;i<4;i++) setMotorDutyPct(i,duty);
          Serial.printf("[TEST] ALL motors → %d%%\n", duty);
          char buf[32]; snprintf(buf,32,"TEST:ALL:%d%%",duty);
          bleNotify(buf);
        } else {
          setMotorDutyPct(testMotor,duty);
          Serial.printf("[TEST] %s → %d%%\n", MOTOR_NAMES[testMotor], duty);
          char buf[48];
          snprintf(buf,48,"TEST:%s:%d%%",MOTOR_NAMES[testMotor],duty);
          bleNotify(buf);
        }
      } else {
        testMode = false;
        Serial.println("[TEST] All motors stopped");
        bleNotify("TEST:STOP");
      }
    }
    return;
  }

  if(s=="IMU"){
    // ── One-shot IMU angle dump ─────────────────────────────
    RawIMU r=readRawIMU();
    float ax=(r.ax-accX_offset)/ACCEL_SENS;
    float ay=(r.ay-accY_offset)/ACCEL_SENS;
    float az=(r.az-accZ_offset)/ACCEL_SENS;
    float roll_a  = atan2f(ay,az)*RAD_TO_DEG;
    float pitch_a = atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;
    Serial.printf("[IMU] cf_roll=%.2f cf_pitch=%.2f | accel_roll=%.2f accel_pitch=%.2f\n",
      cf_roll,cf_pitch,roll_a,pitch_a);
    char buf[80];
    snprintf(buf,80,"IMU:R=%.1f,P=%.1f",cf_roll,cf_pitch);
    bleNotify(buf);
    return;
  }

  if(s=="GYRO"){
    gyroLiveMode=true;
    Serial.println("[GYRO] Live mode ON — tilt the drone, watch signs");
    bleNotify("GYRO:ON");
    return;
  }

  if(s=="GYROSTOP"){
    gyroLiveMode=false;
    Serial.println("[GYRO] Live mode OFF");
    bleNotify("GYRO:OFF");
    return;
  }

  // ── Normal flight packet ──────────────────────────────────
  testMode=false;
  int tI,pI,rI,yI;
  int parsed=sscanf(s.c_str(),"T:%d,P:%d,R:%d,Y:%d",&tI,&pI,&rI,&yI);
  if(parsed<1) return;
  sp_throttle = clamp_i(tI,0,100);
  if(parsed>=2) sp_pitch    = clamp_f((float)pI,-30.f,30.f);
  if(parsed>=3) sp_roll     = clamp_f((float)rI,-30.f,30.f);
  if(parsed>=4) sp_yaw_rate = clamp_f((float)yI/100.f*MAX_YAW_RATE,-MAX_YAW_RATE,MAX_YAW_RATE);
  motorsArmed=true;
}

// ═══════════════════════════════════════════════════════════
//  BLE Callbacks
// ═══════════════════════════════════════════════════════════
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected=true;
    Serial.println("[BLE] Connected");
  }
  void onDisconnect(BLEServer*) override {
    bleConnected=false;
    motorsArmed=false;
    testMode=false;
    gyroLiveMode=false;
    stopAllMotors();
    rollState={}; pitchState={}; yawState={};
    Serial.println("[BLE] Disconnected – motors stopped");
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
void setup(){
  Serial.begin(115200);
  delay(600);
  Serial.println("\n=== ESP32-C3 Drone — PID + Motor Test Mode ===");
  Serial.println("Motors: 0=FL(GPIO2) 1=FR(GPIO3) 2=BL(GPIO4) 3=BR(GPIO5)");
  Serial.println("MPU-6050: +Y=forward(pitch), +X=right(roll), +Z=up(yaw)\n");

  for(int i=0;i<4;i++){
    ledcSetup(i,PWM_FREQ,PWM_RES);
    ledcAttachPin(MOTOR_PINS[i],i);
    ledcWrite(i,0);
  }

  Wire.begin(6,7);
  Wire.setClock(400000);
  mpuWriteReg(0x6B,0x00); delay(100);
  mpuWriteReg(0x1B,0x00);
  mpuWriteReg(0x1C,0x00);
  mpuWriteReg(0x1A,0x03);
  delay(50);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,1,true);
  uint8_t who=Wire.read();
  Serial.printf("[IMU] WHO_AM_I=0x%02X %s\n",who,who==0x68?"OK":"WARNING");

  calibrateIMU();

  // Seed filter
  {
    RawIMU r=readRawIMU();
    float ax=(r.ax-accX_offset)/ACCEL_SENS;
    float ay=(r.ay-accY_offset)/ACCEL_SENS;
    float az=(r.az-accZ_offset)/ACCEL_SENS;
    cf_roll  = atan2f(ay,az)*RAD_TO_DEG;
    cf_pitch = atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;
  }

  // BLE
  BLEDevice::init("ESP32-C3-Drone");
  pServer=BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService* svc=pServer->createService(BLEUUID(SERVICE_UUID),30);

  // Write characteristic (controller → drone)
  pCharWrite=svc->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE|
    BLECharacteristic::PROPERTY_WRITE_NR);
  pCharWrite->setCallbacks(new CharCB());
  pCharWrite->addDescriptor(new BLE2902());

  // Notify characteristic (drone → controller, for IMU feedback)
  pCharNotify=svc->createCharacteristic(
    NOTIFY_UUID,
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
float _gz_live=0;

void loop(){
  unsigned long now=micros();

  // ── Fast path: IMU + filter ──────────────────────────────
  if(now-lastFastUs>=2000){
    float dt=(now-lastFastUs)*1e-6f;
    lastFastUs=now;

    RawIMU r=readRawIMU();
    float ax=(r.ax-accX_offset)/ACCEL_SENS;
    float ay=(r.ay-accY_offset)/ACCEL_SENS;
    float az=(r.az-accZ_offset)/ACCEL_SENS;
    float gx=(r.gx-gyroX_offset)/GYRO_SENS;
    float gy=(r.gy-gyroY_offset)/GYRO_SENS;
    float gz=(r.gz-gyroZ_offset)/GYRO_SENS;
    _gz_live=gz;

    float acc_roll  = atan2f(ay,az)*RAD_TO_DEG;
    float acc_pitch = atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;

    cf_roll  = CF_ALPHA*(cf_roll  + gx*dt) + (1.f-CF_ALPHA)*acc_roll;
    cf_pitch = CF_ALPHA*(cf_pitch + gy*dt) + (1.f-CF_ALPHA)*acc_pitch;

    // ── Gyro live mode print ────────────────────────────────
    if(gyroLiveMode && millis()-lastGyroMs>100){
      lastGyroMs=millis();
      Serial.printf("[GYRO] Roll=%+6.1f° Pitch=%+6.1f° | gx=%+5.1f gy=%+5.1f gz=%+5.1f °/s\n",
        cf_roll,cf_pitch,gx*dt*500,gy*dt*500,gz);
      // Also notify over BLE (compact)
      char buf[60];
      snprintf(buf,60,"GYRO:R=%.1f,P=%.1f",cf_roll,cf_pitch);
      bleNotify(buf);
    }

    // ── Slow path: PID + motors ──────────────────────────────
    if(now-lastSlowUs>=10000){
      float pidDt=(now-lastSlowUs)*1e-6f;
      lastSlowUs=now;

      // Watchdog
      if(motorsArmed && !testMode && (millis()-lastPacketMs>WATCHDOG_MS)){
        Serial.println("[WATCHDOG] Timeout – stopping");
        stopAllMotors();
        motorsArmed=false;
        rollState={}; pitchState={}; yawState={};
      }

      // Test mode: motors already set by parser, skip PID
      if(testMode) return;

      if(bleConnected && motorsArmed){
        float rC=computePID(rollState, rollGains, sp_roll, cf_roll, pidDt);
        float pC=computePID(pitchState,pitchGains,sp_pitch,cf_pitch,pidDt);
        float yC=computePID(yawState,  yawGains,  sp_yaw_rate,_gz_live,pidDt);
        applyMix(sp_throttle,rC,pC,yC);
      }

      // Debug serial every 500 ms
      if(millis()-lastPrintMs>500){
        lastPrintMs=millis();
        Serial.printf("Roll=%+5.1f° Pitch=%+5.1f° | T=%d\n",
          cf_roll,cf_pitch,sp_throttle);
      }
    }
  }
}