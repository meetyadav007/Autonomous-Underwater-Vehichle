#include <Wire.h>
#include <Servo.h>             
#include <SoftwareSerial.h>    
#include "MPU6050.h"           


const uint8_t PIN_FWD_L = 5;
const uint8_t PIN_FWD_R = 6;
const uint8_t PIN_SIDE_L = 9;
const uint8_t PIN_SIDE_R = 10;
const uint8_t PIN_VERT_L = 3;
const uint8_t PIN_VERT_R = 11;

const uint8_t TFL_RX = 2;  
const uint8_t TFL_TX = 4;  


const int PWM_MIN = 1100;
const int PWM_NEU = 1500;   // stop/neutral
const int PWM_MAX = 1900;

const int SURGE_SPEED = 1600;          // forward
const int SWAY_RIGHT_L = 1600;         // side-left thruster forward
const int SWAY_RIGHT_R = 1400;         // side-right thruster reverse
const int SWAY_LEFT_L  = 1400;         // side-left reverse
const int SWAY_LEFT_R  = 1600;         // side-right forward


Servo thrFwdL, thrFwdR, thrSideL, thrSideR, thrVertL, thrVertR;
SoftwareSerial tfl(TFL_RX, TFL_TX);
MPU6050 imu;


const int BASELINE_SAMPLES = 60;  // avg this many samples for "wall" distance
const int N_SMOOTH = 5;          // moving average window for TF-Luna
const int OPEN_DELTA = 60;       // cm above baseline -> "gap/opening"
const unsigned long CONTROL_DT_MS = 50;  // loop timing
const unsigned long FORWARD_TIME_MS = 4000; 
const float SWAY_SPEED_MPS = 0.15;  
const float CENTER_DEADBAND_M = 0.03; // 3 cm tolerance


enum State { ARM_ESC, BASELINE, SWEEP_RIGHT, RECENTER, PASS_THROUGH, UTURN, DONE, FAIL };
State state = ARM_ESC;


unsigned long lastTick = 0;
unsigned long forwardStart = 0;

int baseline = 0;
bool inOpen = false;

float x = 0.0f;               
float xLeft = NAN, xRight = NAN;

float yawDeg = 0.0f;          
float targetYawDeg = 0.0f;    


void writeNeutralAll() {
  thrFwdL.writeMicroseconds(PWM_NEU);
  thrFwdR.writeMicroseconds(PWM_NEU);
  thrSideL.writeMicroseconds(PWM_NEU);
  thrSideR.writeMicroseconds(PWM_NEU);
  thrVertL.writeMicroseconds(PWM_NEU);
  thrVertR.writeMicroseconds(PWM_NEU);
}

void swayRight() {
  thrSideL.writeMicroseconds(SWAY_RIGHT_L);
  thrSideR.writeMicroseconds(SWAY_RIGHT_R);
}

void swayLeft() {
  thrSideL.writeMicroseconds(SWAY_LEFT_L);
  thrSideR.writeMicroseconds(SWAY_LEFT_R);
}

void surgeForward() {
  thrFwdL.writeMicroseconds(SURGE_SPEED);
  thrFwdR.writeMicroseconds(SURGE_SPEED);
}

void yawHoldSimple() {
  
  float err = targetYawDeg - yawDeg;
  
  if (err > 180) err -= 360;
  if (err < -180) err += 360;

  
  int corr = (int)(err * 2.0); 
  corr = constrain(corr, -40, 40);

  int leftPWM  = PWM_NEU + corr; 
  int rightPWM = PWM_NEU - corr;

  
  thrFwdL.writeMicroseconds(constrain(leftPWM,  PWM_MIN, PWM_MAX));
  thrFwdR.writeMicroseconds(constrain(rightPWM, PWM_MIN, PWM_MAX));
}


// Reads one frame if available. Returns distance in cm or -1 if not ready.
int readTFLunaOnce() {
  if (tfl.available() < 9) return -1;

 
  while (tfl.available() >= 1 && tfl.peek() != 0x59) {
    tfl.read();
  }
  if (tfl.available() < 9) return -1;

  uint8_t b0 = tfl.read(); if (b0 != 0x59) return -1;
  uint8_t b1 = tfl.read(); if (b1 != 0x59) return -1;

  uint8_t buf[7];
  for (int i = 0; i < 7; i++) buf[i] = tfl.read();

  uint8_t sum = b0 + b1;
  for (int i = 0; i < 6; i++) sum += buf[i];   // checksum = low byte of bytes 0..7
  if (sum != buf[6]) return -1;                // buf[6] is the frame's checksum byte

  int dist = (int)buf[1] << 8 | buf[0];  // cm
  return dist;
}

// Simple moving average
int smoothDistance() {
  static int q[10];
  static int idx = 0, filled = 0;

  int d = readTFLunaOnce();
  if (d > 0) {
    q[idx] = d;
    idx = (idx + 1) % N_SMOOTH;
    if (filled < N_SMOOTH) filled++;
  }

  if (filled == 0) return -1;
  long s = 0;
  for (int i = 0; i < filled; i++) s += q[i];
  return (int)(s / filled);
}


unsigned long imuPrevMs = 0;

void updateIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (imuPrevMs == 0) ? 0.0f : (now - imuPrevMs) / 1000.0f;
  imuPrevMs = now;

  
  float yawRate = gz / 131.0f;
  yawDeg += yawRate * dt;

  
  if (yawDeg > 180) yawDeg -= 360;
  if (yawDeg < -180) yawDeg += 360;
}


void setup() {
  Serial.begin(115200);

  
  thrFwdL.attach(PIN_FWD_L);
  thrFwdR.attach(PIN_FWD_R);
  thrSideL.attach(PIN_SIDE_L);
  thrSideR.attach(PIN_SIDE_R);
  thrVertL.attach(PIN_VERT_L);
  thrVertR.attach(PIN_VERT_R);
  writeNeutralAll();

  
  tfl.begin(115200);
  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU connection failed! Check wiring.");
  } else {
    Serial.println("IMU OK");
  }

  lastTick = millis();
  imuPrevMs = millis();

  Serial.println("AUV Gate Mission — starting");
}


void loop() {
  unsigned long now = millis();
  if (now - lastTick < CONTROL_DT_MS) return;  
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;

  updateIMU();            

  int d = smoothDistance();
  if (d < 0) return;      

  switch (state) {

    case ARM_ESC: {
      Serial.println("Arming ESCs...");
      writeNeutralAll();
      delay(2000);                 
      targetYawDeg = yawDeg;       
      state = BASELINE;
      break;
    }

    case BASELINE: {
      static long sum = 0;
      static int cnt = 0;
      int m = d;
      if (m > 0) {
        sum += m;
        cnt++;
      }
      if (cnt >= BASELINE_SAMPLES) {
        baseline = sum / cnt;
        Serial.print("Baseline (wall) = ");
        Serial.print(baseline);
        Serial.println(" cm");
        state = SWEEP_RIGHT;
      }
      break;
    }

    case SWEEP_RIGHT: {
      
      swayRight();
      
      yawHoldSimple();

      x += SWAY_SPEED_MPS * dt; 

      bool openNow = (d > baseline + OPEN_DELTA);

      // Detect near->open = Left Edge
      static bool wasOpen = false;
      if (openNow && !wasOpen && isnan(xLeft)) {
        xLeft = x;
        Serial.print("Left edge @ ");
        Serial.print(xLeft, 2);
        Serial.println(" m");
      }

      // Detect open->near = Right Edge
      if (!openNow && wasOpen && !isnan(xLeft) && isnan(xRight)) {
        xRight = x;
        Serial.print("Right edge @ ");
        Serial.print(xRight, 2);
        Serial.println(" m");
      }
      wasOpen = openNow;

      
      if (!isnan(xLeft) && !isnan(xRight)) {
        writeNeutralAll();
        state = RECENTER;
      }

      
      static unsigned long t0 = now;
      if (now - t0 > 20000UL && (isnan(xLeft) || isnan(xRight))) {
        Serial.println("Sweep timeout — FAIL");
        writeNeutralAll();
        state = FAIL;
      }
      break;
    }

    case RECENTER: {
      float xCenter = 0.5f * (xLeft + xRight);
      float err = xCenter - x; // + => need to move RIGHT

      if (fabs(err) > CENTER_DEADBAND_M) {
        if (err > 0) {
          swayRight();
          x += SWAY_SPEED_MPS * dt;
        } else {
          swayLeft();
          x -= SWAY_SPEED_MPS * dt;
        }
        yawHoldSimple();
      } else {
        writeNeutralAll();
        Serial.print("Centered at x = ");
        Serial.print(x, 2);
        Serial.println(" m — moving forward");
        forwardStart = now;
        state = PASS_THROUGH;
      }
      break;
    }

    case PASS_THROUGH: {
      
      surgeForward();
      

      if (now - forwardStart > FORWARD_TIME_MS) {
        writeNeutralAll();
        Serial.println("Gate passed — starting U-Turn");
       
        float newTarget = yawDeg + 180.0f;
        if (newTarget > 180) newTarget -= 360;
        targetYawDeg = newTarget;
        state = UTURN;
      }
      break;
    }

    case UTURN: {
      
      float err = targetYawDeg - yawDeg;
      if (err > 180) err -= 360;
      if (err < -180) err += 360;

    
      if (fabs(err) > 5.0f) {
        
        int yawCmd = (err > 0) ? 1600 : 1400; 
        thrFwdL.writeMicroseconds(yawCmd);
        thrFwdR.writeMicroseconds(3000 - yawCmd); 
      } else {
        writeNeutralAll();
        Serial.println("U-Turn complete — DONE");
        state = DONE;
      }
      break;
    }

    case DONE: {
      writeNeutralAll();
      // Next taskS
      break;
    }

    case FAIL: {
      writeNeutralAll();
      Serial.println("Mission FAILED — Gate not found.");
      break;
    }
  }
}
