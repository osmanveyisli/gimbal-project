
#include <Wire.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

// TWO-MPU GEOMETRIC STABILIZATION
// Platform MPU: 0x68
// Base MPU:     0x69
// Roll servo:   D10
// Pitch servo:  D9
//
// Automatic control is OFF at startup.

const byte PLATFORM = 0;
const byte BASE = 1;

const byte MPU_ADDR[2] = {0x68, 0x69};
const byte ROLL_PIN = 10;
const byte PITCH_PIN = 9;

// Your measured accelerometer calibration
const float ACC_BIAS[2][3] = {
  {0.015863f, 0.002396f, -0.027264f},
  {0.011293f, -0.008412f, 0.105548f}
};

const float ACC_SCALE[2][3] = {
  {1.001796f, 1.000605f, 0.996350f},
  {1.006142f, 0.999601f, 0.988342f}
};

// Orientation filter
const float FILTER_KP = 2.0f;

// Geometric joint angles are provisionally treated as 1:1 servo-command degrees.
// This is an actuator-model assumption, not a verified calibration.
// Keep feedforward and feedback gains available for tuning.

const float ROLL_KP = 0.30f;
const float ROLL_KD = 0.050f;

const float PITCH_KP = 0.30f;
const float PITCH_KD = 0.050f;

const float SLEW_DEG_PER_SEC = 80.0f;

const float ROLL_MIN = 45.0f;
const float ROLL_MAX = 180.0f;

const float PITCH_MIN = 15.0f;
const float PITCH_MAX = 165.0f;

const float MAX_BASE_ROLL = 90.0f;
const float MAX_BASE_PITCH = 90.0f;

const float ROLL_FF_GAIN = 0.85f;
const float PITCH_FF_GAIN = 0.85f;

const float CENTER = 90.0f;

// Target angle limits; start conservatively and verify physical travel.
const float MAX_TARGET_ROLL = 30.0f;
const float MAX_TARGET_PITCH = 30.0f;

// If the platform MPU is tilted relative to the actual platform,
// measure its roll/pitch while the PLATFORM is physically level,
// then put those readings here. Zero means sensor axes aligned.
const float PLATFORM_LEVEL_SENSOR_ROLL = 0.0f;
const float PLATFORM_LEVEL_SENSOR_PITCH = 0.0f;

// ==================================
// DATA TYPES
// ==================================

struct Vec3 {
  float x, y, z;
};

struct Mat3 {
  float m[3][3];
};

struct SensorState {
  Mat3 R;
  Vec3 gyroBias;
  Vec3 gyroFRD;

  float accelMagnitude;
  float roll;
  float pitch;
};

SensorState imu[2];

Servo rollServo;
Servo pitchServo;

enum Mode {
  HOLD_MODE,
  ROLL_MODE,
  PITCH_MODE,
  BOTH_MODE
};

Mode mode = HOLD_MODE;

// References saved at startup or by 'z'
float baseRollRef;
float basePitchRef;

float platformRollRef;
float platformPitchRef;

// Geometric joint angles at reference position
float geoRollRef;
float geoPitchRef;

// Actual platform sensor roll/pitch measured at the last 'z' reference.
float referencePlatformRoll;
float referencePlatformPitch;

// Servo commands at the moment we calibrated the reference.
float referenceRollCommand = CENTER;
float referencePitchCommand = CENTER;

// User's requested PLATFORM angles relative to gravity,
// after applying platform-MPU mounting offsets.
float targetRoll = 0.0f;
float targetPitch = 0.0f;

// Simple non-blocking parser for commands such as 't 15 20'.
char serialLine[40];
byte serialLineLength = 0;

// Servo positions
float rollCommand = CENTER;
float pitchCommand = CENTER;

float rollGoal = CENTER;
float pitchGoal = CENTER;

// Sensor-fault handling is separate from the user's chosen control mode.
// A missing sample immediately freezes servo updates (no stale control data).
// 1 second of unresolved read trouble is considered a sustained outage.
// After a sustained outage, require 0.8 seconds of clean reads to resume.
const unsigned long SENSOR_OUTAGE_HOLD_MS = 1000UL;
const unsigned long SENSOR_STABLE_RECOVERY_MS = 800UL;
const unsigned long SHORT_GLITCH_MAX_MS = 100UL;
const byte QUICK_RECOVERY_READS = 5;

byte goodReads = 0;

bool sensorPaused = false;
bool sustainedSensorFault = false;
bool recoveryStarted = false;
byte recoveryGoodReads = 0;
unsigned long sensorFaultSinceMs = 0;
unsigned long recoverySinceMs = 0;

unsigned long lastSensorUs;
unsigned long lastControlUs;
unsigned long lastPrintMs;
unsigned long lastFaultMs = 0;

// ==================================
// BASIC MATHEMATICS
// ==================================

float clampF(float v, float low, float high) {
  if (v < low) return low;
  if (v > high) return high;
  return v;
}

float differenceDeg(float a, float b) {
  float d = a - b;

  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;

  return d;
}

float length3(Vec3 v) {
  return sqrt(
    v.x*v.x +
    v.y*v.y +
    v.z*v.z
  );
}

Vec3 cross3(Vec3 a, Vec3 b) {
  return {
    a.y*b.z - a.z*b.y,
    a.z*b.x - a.x*b.z,
    a.x*b.y - a.y*b.x
  };
}

// ==================================
// ROTATION MATRICES
// ==================================

Mat3 transpose3(Mat3 a) {
  Mat3 t = {};

  for (byte i = 0; i < 3; i++)
    for (byte j = 0; j < 3; j++)
      t.m[i][j] = a.m[j][i];

  return t;
}

Mat3 multiply3(Mat3 a, Mat3 b) {
  Mat3 c = {};

  for (byte i = 0; i < 3; i++)
    for (byte j = 0; j < 3; j++)
      for (byte k = 0; k < 3; k++)
        c.m[i][j] += a.m[i][k] * b.m[k][j];

  return c;
}

Vec3 apply3(Mat3 a, Vec3 v) {
  return {
    a.m[0][0]*v.x +
    a.m[0][1]*v.y +
    a.m[0][2]*v.z,

    a.m[1][0]*v.x +
    a.m[1][1]*v.y +
    a.m[1][2]*v.z,

    a.m[2][0]*v.x +
    a.m[2][1]*v.y +
    a.m[2][2]*v.z
  };
}

Mat3 rotX(float a) {
  float c = cos(a);
  float s = sin(a);

  return {{
    {1, 0, 0},
    {0, c, -s},
    {0, s, c}
  }};
}

Mat3 rotY(float a) {
  float c = cos(a);
  float s = sin(a);

  return {{
    {c, 0, s},
    {0, 1, 0},
    {-s, 0, c}
  }};
}

// Rodrigues' formula
Mat3 rotVector(Vec3 v) {
  float x = v.x;
  float y = v.y;
  float z = v.z;

  float t2 = x*x + y*y + z*z;
  float t = sqrt(t2);

  float A = t < 0.001f
    ? 1 - t2/6.0f
    : sin(t)/t;

  float B = t < 0.001f
    ? 0.5f - t2/24.0f
    : (1-cos(t))/t2;

  return {{
    {
      1-B*(y*y+z*z),
      B*x*y-A*z,
      B*x*z+A*y
    },
    {
      B*x*y+A*z,
      1-B*(x*x+z*z),
      B*y*z-A*x
    },
    {
      B*x*z-A*y,
      B*y*z+A*x,
      1-B*(x*x+y*y)
    }
  }};
}

// ==================================
// SENSOR COORDINATES
// ==================================

// Forward-Right-Down convention.
// Base sensor mapping is approximate.

Vec3 localFRD(byte id, Vec3 v) {
  if (id == PLATFORM) {
    return {-v.x, v.y, -v.z};
  }

  return {-v.y, -v.x, -v.z};
}

Vec3 calibratedAcc(byte id, Vec3 a) {
  return {
    (a.x-ACC_BIAS[id][0])*ACC_SCALE[id][0],
    (a.y-ACC_BIAS[id][1])*ACC_SCALE[id][1],
    (a.z-ACC_BIAS[id][2])*ACC_SCALE[id][2]
  };
}

// ==================================
// MPU COMMUNICATION
// ==================================

bool writeReg(
  byte address,
  byte reg,
  byte value
) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

int16_t read16() {
  uint16_t hi = (uint8_t)Wire.read();
  uint16_t lo = (uint8_t)Wire.read();

  return (int16_t)((hi << 8) | lo);
}

bool readMPU(byte id, Vec3 &a, Vec3 &g) {
  byte address = MPU_ADDR[id];

  Wire.beginTransmission(address);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (
    Wire.requestFrom(
      address, (byte)14, (byte)true
    ) != 14
  ) {
    return false;
  }

  int16_t ax = read16();
  int16_t ay = read16();
  int16_t az = read16();

  read16(); // Temperature

  int16_t gx = read16();
  int16_t gy = read16();
  int16_t gz = read16();

  a = {
    ax/16384.0f,
    ay/16384.0f,
    az/16384.0f
  };

  g = {
    gx/131.0f,
    gy/131.0f,
    gz/131.0f
  };

  return true;
}

// ==================================
// GYROSCOPE CALIBRATION
// ==================================

bool calibrateGyros() {
  for (byte id = 0; id < 2; id++) {
    imu[id].gyroBias = {0,0,0};
  }

  Serial.println(
    F("Hold gimbal completely still")
  );

  delay(1000);

  for (int i = 0; i < 500; i++) {
    for (byte id = 0; id < 2; id++) {
      Vec3 a, g;

      if (!readMPU(id, a, g))
        return false;

      imu[id].gyroBias.x += g.x;
      imu[id].gyroBias.y += g.y;
      imu[id].gyroBias.z += g.z;
    }

    delay(5);
  }

  for (byte id = 0; id < 2; id++) {
    imu[id].gyroBias.x /= 500.0f;
    imu[id].gyroBias.y /= 500.0f;
    imu[id].gyroBias.z /= 500.0f;
  }

  return true;
}

// ==================================
// ORIENTATION ESTIMATION
// ==================================

void readAngles(byte id) {
  imu[id].roll = atan2(
    imu[id].R.m[2][1],
    imu[id].R.m[2][2]
  ) * RAD_TO_DEG;

  imu[id].pitch = asin(
    clampF(
      -imu[id].R.m[2][0],
      -1.0f,
      1.0f
    )
  ) * RAD_TO_DEG;
}

// ==================================
// NEW: GEOMETRIC JOINT SOLVER
// ==================================

// Base matrix -> gravity vector
// -> roll and pitch joint angles.
//
// Ideal model:
// R_joint = Rx(roll) * Ry(pitch)
//
// These are physical angles in degrees,
// NOT servo command angles.

// Return the world-down (gravity) direction expressed in this
// sensor's Forward-Right-Down coordinate frame.
Vec3 gravityInSensor(byte id) {
  Vec3 g = apply3(transpose3(imu[id].R), {0, 0, 1});
  float n = length3(g);
  if (n > 0.1f) {
    g.x /= n;
    g.y /= n;
    g.z /= n;
  }
  return g;
}

// Convert a gravity-referenced roll/pitch into the corresponding
// down vector in the platform's sensor frame.
// This uses the SAME Euler-angle convention as readAngles().
Vec3 downVectorAt(float rollDeg, float pitchDeg) {
  float r = rollDeg * (float)DEG_TO_RAD;
  float p = pitchDeg * (float)DEG_TO_RAD;
  return {-sin(p), sin(r)*cos(p), cos(r)*cos(p)};
}

// Inverse kinematics for R_joint = Rx(roll) * Ry(pitch).
// gBase = R_joint * gPlatformTarget.
// The solver uses GRAVITY VECTORS, so yaw doesn't need a magnetometer.
// Requires moderately upright geometry (not a gimbal inversion).
bool geometricJointAnglesFor(
  Vec3 gPlatformTarget, float &rollDeg, float &pitchDeg
) {
  Vec3 g = gravityInSensor(BASE);
  float n = length3(g);
  if (n < 0.8f || g.z < 0.20f) return false;

  float yz = sqrt(g.y*g.y + g.z*g.z);
  if (yz < 0.20f) return false;

  // First solve the roll rotation from the Y component.
  // gTarget.y = gBase.y*cos(roll) + gBase.z*sin(roll).
  float fraction = gPlatformTarget.y / yz;
  if (fraction < -1.001f || fraction > 1.001f) return false;
  float roll = asin(clampF(fraction, -1.0f, 1.0f)) - atan2(g.y, g.z);

  // Then solve pitch as the remaining rotation in the X-Z plane.
  float zAfterRoll = -sin(roll)*g.y + cos(roll)*g.z;
  float pitch = atan2(
      g.x*gPlatformTarget.z - zAfterRoll*gPlatformTarget.x,
      g.x*gPlatformTarget.x + zAfterRoll*gPlatformTarget.z
  );

  rollDeg = roll * (float)RAD_TO_DEG;
  pitchDeg = pitch * (float)RAD_TO_DEG;

  // Avoid ambiguous inverted-configuration branches.
  return fabs(rollDeg) <= 89.0f && fabs(pitchDeg) <= 89.0f;
}

bool geometricJointAngles(float &rollDeg, float &pitchDeg) {
  return geometricJointAnglesFor({0, 0, 1}, rollDeg, pitchDeg);
}

// ==================================
// REFERENCE ORIENTATIONS
// ==================================

// Capture a new mechanical/servo reference and make the current
// platform attitude the new desired angle (old 'z' behavior).
// Only call after both motors and the frame have settled.
bool saveReferences() {
  baseRollRef = imu[BASE].roll;
  basePitchRef = imu[BASE].pitch;
  platformRollRef = imu[PLATFORM].roll;
  platformPitchRef = imu[PLATFORM].pitch;

  referencePlatformRoll = imu[PLATFORM].roll;
  referencePlatformPitch = imu[PLATFORM].pitch;
  referenceRollCommand = rollCommand;
  referencePitchCommand = pitchCommand;

  Vec3 currentDown = gravityInSensor(PLATFORM);
  if (!geometricJointAnglesFor(currentDown, geoRollRef, geoPitchRef))
    return false;

  targetRoll = referencePlatformRoll - PLATFORM_LEVEL_SENSOR_ROLL;
  targetPitch = referencePlatformPitch - PLATFORM_LEVEL_SENSOR_PITCH;
  return true;
}

bool initializeOrientations() {
  for (byte id = 0; id < 2; id++) {
    Vec3 aRaw, gRaw;

    if (!readMPU(id, aRaw, gRaw))
      return false;

    Vec3 a = localFRD(
      id,
      calibratedAcc(id, aRaw)
    );

    float n = length3(a);

    if (n < 0.85f || n > 1.15f)
      return false;

    float roll = atan2(-a.y, -a.z);

    float pitch = atan2(
      a.x,
      sqrt(a.y*a.y + a.z*a.z)
    );

    imu[id].R = multiply3(
      rotY(pitch),
      rotX(roll)
    );

    imu[id].gyroFRD = {0,0,0};
    imu[id].accelMagnitude = n;

    readAngles(id);
  }

  return saveReferences();
}

void updateOrientation(
  byte id,
  Vec3 rawAcc,
  Vec3 rawGyro,
  float dt
) {
  SensorState &s = imu[id];

  Vec3 a = localFRD(
    id,
    calibratedAcc(id, rawAcc)
  );

  s.accelMagnitude = length3(a);

  Vec3 rawG = {
    rawGyro.x - s.gyroBias.x,
    rawGyro.y - s.gyroBias.y,
    rawGyro.z - s.gyroBias.z
  };

  s.gyroFRD = localFRD(id, rawG);

  Vec3 w = {
    s.gyroFRD.x * (float)DEG_TO_RAD,
    s.gyroFRD.y * (float)DEG_TO_RAD,
    s.gyroFRD.z * (float)DEG_TO_RAD
  };

  if (
    s.accelMagnitude > 0.80f &&
    s.accelMagnitude < 1.20f
  ) {
    Vec3 measured = {
      -a.x/s.accelMagnitude,
      -a.y/s.accelMagnitude,
      -a.z/s.accelMagnitude
    };

    Vec3 predicted = apply3(
      transpose3(s.R),
      {0,0,1}
    );

    Vec3 error = cross3(
      measured,
      predicted
    );

    w.x += FILTER_KP*error.x;
    w.y += FILTER_KP*error.y;
    w.z += FILTER_KP*error.z;
  }

  s.R = multiply3(
    s.R,
    rotVector({
      w.x*dt,
      w.y*dt,
      w.z*dt
    })
  );

  readAngles(id);
}

// ==================================
// FAULT HANDLING
// ==================================

// Mechanical/geometric faults remain latched in manual HOLD.
// Sensor communication failures use a separate, recoverable pause;
// servo PWM keeps holding the last commanded position.

void checkSensorFaultDuration() {
  if (sensorPaused && !sustainedSensorFault &&
      millis() - sensorFaultSinceMs >= SENSOR_OUTAGE_HOLD_MS) {
    sustainedSensorFault = true;
    if (mode != HOLD_MODE) {
      Serial.println(F("SENSOR HOLD: 1s unreliable; awaiting 800ms clean readings"));
    }
  }
}

void noteBadSensorSample() {
  unsigned long now = millis();

  if (!sensorPaused) {
    sensorPaused = true;
    sustainedSensorFault = false;
    sensorFaultSinceMs = now;
    if (mode != HOLD_MODE) {
      Serial.println(F("SENSOR PAUSE: servo commands frozen; checking recovery"));
    }
  }

  goodReads = 0;
  recoveryGoodReads = 0;
  recoveryStarted = false;
  checkSensorFaultDuration();

  if (now - lastFaultMs >= 1000UL) {
    Serial.println(F("I2C/timing sample rejected"));
    lastFaultMs = now;
  }
}

void noteGoodSensorSample() {
  if (goodReads < 100) ++goodReads;
  if (!sensorPaused) return;

  checkSensorFaultDuration();
  if (!recoveryStarted) {
    recoveryStarted = true;
    recoverySinceMs = millis();
    recoveryGoodReads = 0;
  }
  if (recoveryGoodReads < 255) ++recoveryGoodReads;

  // Brief (<100ms) glitches recover after 5 consecutive valid samples.
  // Larger outages (including >1s) require 800ms clean data so the
  // gravity-based orientation estimator can settle before servo updates.
  bool quickGlitch = !sustainedSensorFault &&
      (millis() - sensorFaultSinceMs <= SHORT_GLITCH_MAX_MS) &&
      (recoveryGoodReads >= QUICK_RECOVERY_READS);
  bool stableRecovery =
      (millis() - recoverySinceMs >= SENSOR_STABLE_RECOVERY_MS) &&
      (goodReads >= 20);
  bool recovered = quickGlitch || stableRecovery;

  if (recovered) {
    sensorPaused = false;
    sustainedSensorFault = false;
    recoveryStarted = false;
    recoveryGoodReads = 0;
    lastControlUs = micros(); // Never integrate a large stale control dt.
    if (mode != HOLD_MODE) {
      Serial.println(F("SENSORS RECOVERED: previous AUTO mode resumed"));
    } else {
      Serial.println(F("SENSORS RECOVERED: manual HOLD unchanged"));
    }
  }
}

void stopAuto(
  const __FlashStringHelper *reason
) {
  mode = HOLD_MODE;

  Serial.print(
    F("AUTO OFF (BOTH HOLD): ")
  );

  Serial.println(reason);
}

bool stillEnough() {
  if (sensorPaused || goodReads < 20)
    return false;

  for (byte id = 0; id < 2; id++) {
    if (
      fabs(imu[id].gyroFRD.x) > 1.5f ||
      fabs(imu[id].gyroFRD.y) > 1.5f ||
      fabs(imu[id].gyroFRD.z) > 1.5f
    ) {
      return false;
    }
  }

  return true;
}

// ==================================
// OPERATING MODES
// ==================================

void enterMode(Mode requested) {
  // Arming can occur while the gimbal is moving in the user's hand.
  // No stillEnough() requirement here; it remains for the 'z' command.
  if (sensorPaused || goodReads < 20) {
    Serial.println(F("ARM BLOCKED: wait for healthy sensor readings"));
    return;
  }

  float dr = differenceDeg(
    imu[BASE].roll,
    baseRollRef
  );

  float dp = differenceDeg(
    imu[BASE].pitch,
    basePitchRef
  );

  if (
    fabs(dr) > MAX_BASE_ROLL ||
    fabs(dp) > MAX_BASE_PITCH
  ) {
    Serial.println(
      F("ARM BLOCKED: base tilt outside limits")
    );
    return;
  }

  mode = requested;

  Serial.println(
    F("AUTO ON; s stops BOTH automatic updates")
  );
}

void reportTarget() {
  Serial.print(F("TARGET: roll="));
  Serial.print(targetRoll, 1);
  Serial.print(F(" pitch="));
  Serial.println(targetPitch, 1);
}

bool setTarget(float rollDeg, float pitchDeg) {
  if (sensorPaused || goodReads < 20) {
    Serial.println(F("TARGET BLOCKED: sensor data not healthy"));
    return false;
  }

  if (!(rollDeg >= -MAX_TARGET_ROLL && rollDeg <= MAX_TARGET_ROLL &&
        pitchDeg >= -MAX_TARGET_PITCH && pitchDeg <= MAX_TARGET_PITCH)) {
    Serial.println(F("TARGET REJECTED: outside +/-30 degree target limits"));
    return false;
  }

  Vec3 requested = downVectorAt(
      rollDeg + PLATFORM_LEVEL_SENSOR_ROLL,
      pitchDeg + PLATFORM_LEVEL_SENSOR_PITCH
  );
  float r, p;
  if (!geometricJointAnglesFor(requested, r, p)) {
    Serial.println(F("TARGET REJECTED: unreachable geometry"));
    return false;
  }

  targetRoll = rollDeg;
  targetPitch = pitchDeg;
  reportTarget();
  return true;
}

// Called for single-letter commands. These work with or without
// an explicit newline from the Serial Monitor.
void singleCommand(char c) {
  if (c == 's') {
    if (mode != HOLD_MODE) stopAuto(F("manual stop"));
    else Serial.println(F("Already in HOLD"));
  }
  else if (c == '0' && mode == HOLD_MODE) {
    rollCommand = CENTER;
    pitchCommand = CENTER;
    rollGoal = CENTER;
    pitchGoal = CENTER;
    rollServo.write(90);
    pitchServo.write(90);
    Serial.println(F("Centered; allow settling, then send z"));
  }
  else if (c == 'z' && mode == HOLD_MODE) {
    if (!stillEnough()) {
      Serial.println(F("Hold still before capturing z"));
      return;
    }
    if (saveReferences()) {
      Serial.println(F("Current position captured as servo + target reference"));
      reportTarget();
    } else {
      Serial.println(F("REFERENCE FAILED: check orientation"));
    }
  }
  else if (c == 'l' || c == 'L') {
    setTarget(0.0f, 0.0f);
  }
  else if (c == 'p' && mode == HOLD_MODE) enterMode(PITCH_MODE);
  else if (c == 'r' && mode == HOLD_MODE) enterMode(ROLL_MODE);
  else if (c == 'a' && mode == HOLD_MODE) enterMode(BOTH_MODE);
  else {
    Serial.println(F("Unknown command or send s first"));
  }
}

// Non-blocking parser, unlike Serial.parseFloat(), which could
// pause the control loop waiting for missing input.
void handleLine(char *line) {
  char *p = line;
  while (*p == ' ' || *p == '\t') ++p;
  if (!*p) return;

  if (*p != 't' && *p != 'T') {
    Serial.println(F("Unknown command: use t <roll> <pitch>"));
    return;
  }

  ++p;
  if (*p != ' ' && *p != '\t') {
    Serial.println(F("Use t 15 20"));
    return;
  }
  while (*p == ' ' || *p == '\t') ++p;

  char *end;
  double roll = strtod(p, &end);
  if (end == p) {
    Serial.println(F("Missing roll angle; use t 15 20"));
    return;
  }
  p = end;
  while (*p == ' ' || *p == '\t') ++p;

  double pitch = strtod(p, &end);
  if (end == p) {
    Serial.println(F("Missing pitch angle; use t 15 20"));
    return;
  }
  p = end;
  while (*p == ' ' || *p == '\t') ++p;

  if (*p != '\0') {
    Serial.println(F("Unexpected text; use t 15 20"));
    return;
  }
  setTarget((float)roll, (float)pitch);
}

void processSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        handleLine(serialLine);
        serialLineLength = 0;
      }
      continue;
    }

    // Keep the old one-character commands compatible
    // with Serial Monitor set to "No line ending".
    if (serialLineLength == 0 &&
        (c == 's' || c == '0' || c == 'z' || c == 'l' || c == 'L' ||
         c == 'p' || c == 'r' || c == 'a')) {
      singleCommand(c);
      continue;
    }

    if (serialLineLength < sizeof(serialLine)-1) {
      serialLine[serialLineLength++] = c;
    } else {
      serialLineLength = 0;
      Serial.println(F("Serial line too long; command discarded"));
    }
  }
}

// ==================================
// GEOMETRIC + PD CONTROLLER
// ==================================

void updateController(float dt) {
  if (mode == HOLD_MODE || sensorPaused)
    return;

  // A scheduler hiccup should not permanently disarm the controller.
  // Never compute a large servo move using stale data.
  if (dt <= 0 || dt > 0.10f) {
    noteBadSensorSample();
    return;
  }

  // High linear acceleration is not by itself an MPU failure.
  // The orientation filter independently ignores accelerometer correction
  // when acceleration magnitude is outside 0.80g..1.20g, but still integrates
  // gyro measurements. Do not stop automatic control for normal hand motion.

  float dr = differenceDeg(
    imu[BASE].roll,
    baseRollRef
  );

  float dp = differenceDeg(
    imu[BASE].pitch,
    basePitchRef
  );

  // Check both base-axis limits.
  if (
    fabs(dr) > MAX_BASE_ROLL ||
    fabs(dp) > MAX_BASE_PITCH
  ) {
    stopAuto(F("base tilt exceeds limit"));
    return;
  }

  // Geometric joint targets for two platform attitudes:
  // 1) The attitude held when z/reference was captured,
  // 2) The user's requested gravity-relative attitude.
  // Keep FF gain 0.85 for BASE movement, but use 1.0 for a
  // deliberate target change so a requested 15deg isn't treated
  // as only 85% of its value.
  float geoHoldR, geoHoldP, geoWantR, geoWantP;
  Vec3 holdDown = downVectorAt(referencePlatformRoll, referencePlatformPitch);
  Vec3 wantDown = downVectorAt(
      targetRoll + PLATFORM_LEVEL_SENSOR_ROLL,
      targetPitch + PLATFORM_LEVEL_SENSOR_PITCH
  );

  if (!geometricJointAnglesFor(holdDown, geoHoldR, geoHoldP) ||
      !geometricJointAnglesFor(wantDown, geoWantR, geoWantP)) {
    stopAuto(F("geometric target unreachable"));
    return;
  }

  // Calculate both targets BEFORE writing to either motor.
  float newRollGoal = rollCommand;
  float newPitchGoal = pitchCommand;

  if (mode == ROLL_MODE || mode == BOTH_MODE) {
    float error = differenceDeg(
        imu[PLATFORM].roll,
        targetRoll + PLATFORM_LEVEL_SENSOR_ROLL
    );

    newRollGoal = referenceRollCommand
      + ROLL_FF_GAIN * differenceDeg(geoHoldR, geoRollRef)
      + differenceDeg(geoWantR, geoHoldR)
      - ROLL_KP * error
      - ROLL_KD * imu[PLATFORM].gyroFRD.x;
  }

  if (mode == PITCH_MODE || mode == BOTH_MODE) {
    float error = differenceDeg(
        imu[PLATFORM].pitch,
        targetPitch + PLATFORM_LEVEL_SENSOR_PITCH
    );

    newPitchGoal = referencePitchCommand
      + PITCH_FF_GAIN * differenceDeg(geoHoldP, geoPitchRef)
      + differenceDeg(geoWantP, geoHoldP)
      - PITCH_KP * error
      - PITCH_KD * imu[PLATFORM].gyroFRD.y;
  }

  // -------- SHARED LIMIT CHECK --------

  // If EITHER active target is outside
  // its allowed range, both automatic
  // controllers stop BEFORE any write.

  if (
    (mode == ROLL_MODE || mode == BOTH_MODE) &&
    (
      newRollGoal < ROLL_MIN ||
      newRollGoal > ROLL_MAX
    )
  ) {
    stopAuto(
      F("roll target outside servo limits")
    );
    return;
  }

  if (
    (mode == PITCH_MODE || mode == BOTH_MODE) &&
    (
      newPitchGoal < PITCH_MIN ||
      newPitchGoal > PITCH_MAX
    )
  ) {
    stopAuto(
      F("pitch target outside servo limits")
    );
    return;
  }

  // Both targets have passed checks.
  rollGoal = newRollGoal;
  pitchGoal = newPitchGoal;

  float maxStep =
    SLEW_DEG_PER_SEC * dt;

  if (
    mode == ROLL_MODE ||
    mode == BOTH_MODE
  ) {
    rollCommand += clampF(
      rollGoal - rollCommand,
      -maxStep,
      maxStep
    );
  }

  if (
    mode == PITCH_MODE ||
    mode == BOTH_MODE
  ) {
    pitchCommand += clampF(
      pitchGoal - pitchCommand,
      -maxStep,
      maxStep
    );
  }

  // Send valid commands.
  if (
    mode == ROLL_MODE ||
    mode == BOTH_MODE
  ) {
    rollServo.write(
      (int)(rollCommand + 0.5f)
    );
  }

  if (
    mode == PITCH_MODE ||
    mode == BOTH_MODE
  ) {
    pitchServo.write(
      (int)(pitchCommand + 0.5f)
    );
  }
}

// ==================================
// SERIAL OUTPUT
// ==================================

void printStatus() {
  Serial.print(
    mode == HOLD_MODE ? F("HOLD") :
    mode == ROLL_MODE ? F("ROLL") :
    mode == PITCH_MODE ? F("PITCH") :
    F("BOTH")
  );

  if (sensorPaused) {
    Serial.print(sustainedSensorFault
      ? F("/SENSOR_HOLD") : F("/SENSOR_PAUSE"));
  }

  Serial.print(F(" | baseR="));
  Serial.print(
    differenceDeg(
      imu[BASE].roll,
      baseRollRef
    ), 1
  );

  Serial.print(F(" baseP="));
  Serial.print(
    differenceDeg(
      imu[BASE].pitch,
      basePitchRef
    ), 1
  );

  Serial.print(F(" | errR="));
  Serial.print(
    differenceDeg(
      imu[PLATFORM].roll,
      targetRoll + PLATFORM_LEVEL_SENSOR_ROLL
    ), 1
  );

  Serial.print(F(" errP="));
  Serial.print(
    differenceDeg(
      imu[PLATFORM].pitch,
      targetPitch + PLATFORM_LEVEL_SENSOR_PITCH
    ), 1
  );

  Serial.print(F(" | targetR="));
  Serial.print(targetRoll, 1);

  Serial.print(F(" targetP="));
  Serial.print(targetPitch, 1);

  Serial.print(F(" | goalR="));
  Serial.print(rollGoal, 1);

  Serial.print(F(" goalP="));
  Serial.print(pitchGoal, 1);

  Serial.print(F(" | cmdR="));
  Serial.print(rollCommand, 1);

  Serial.print(F(" cmdP="));
  Serial.print(pitchCommand, 1);

  Serial.print(F(" | acc="));
  Serial.print(imu[BASE].accelMagnitude, 2);

  Serial.print(',');

  Serial.println(
    imu[PLATFORM].accelMagnitude, 2
  );
}

// ==================================
// SETUP
// ==================================

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(300);

  for (byte id = 0; id < 2; id++) {
    byte addr = MPU_ADDR[id];

    if (
      !writeReg(addr, 0x6B, 0x01) ||
      !writeReg(addr, 0x1B, 0x00) ||
      !writeReg(addr, 0x1C, 0x00)
    ) {
      Serial.println(F("MPU init FAILED"));

      while (true) delay(1000);
    }
  }

  rollServo.attach(ROLL_PIN);
  pitchServo.attach(PITCH_PIN);

  rollServo.write(90);
  pitchServo.write(90);

  delay(1500);

  if (
    !calibrateGyros() ||
    !initializeOrientations()
  ) {
    Serial.println(
      F("Sensor calibration FAILED")
    );

    rollServo.detach();
    pitchServo.detach();

    while (true) delay(1000);
  }

  lastSensorUs = micros();
  lastControlUs = micros();
  lastPrintMs = millis();

  Serial.println(F("READY / HOLD"));

  Serial.println(
    F("0=center z=hold-now l=level t 15 20=target r=roll p=pitch a=both s=stop")
  );
  Serial.println(F("For t 15 20, use Serial Monitor with Newline enabled."));
  reportTarget();
}

// ==================================
// MAIN LOOP
// ==================================

void loop() {
  Vec3 a[2], g[2];

  // A failed I2C read gives NO new orientation information.
  // Hold last servo commands; do not use stale sensor data.
  bool platformOK = readMPU(PLATFORM, a[PLATFORM], g[PLATFORM]);
  bool baseOK = platformOK && readMPU(BASE, a[BASE], g[BASE]);

  if (!platformOK || !baseOK) {
    noteBadSensorSample();
    processSerial(); // 's' must work even while I2C reads are failing.
    lastSensorUs = micros();
    lastControlUs = micros();
    delay(3);
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastSensorUs) * 0.000001f;
  lastSensorUs = now;

  if (dt <= 0.0f || dt >= 0.10f) {
    noteBadSensorSample();
    processSerial();
    lastControlUs = micros();
    delay(3);
    return;
  }

  updateOrientation(PLATFORM, a[PLATFORM], g[PLATFORM], dt);
  updateOrientation(BASE, a[BASE], g[BASE], dt);
  noteGoodSensorSample();

  processSerial();

  unsigned long controlNow = micros();
  if (controlNow - lastControlUs >= 20000UL) {
    float controlDt = (controlNow - lastControlUs) * 0.000001f;
    lastControlUs = controlNow;
    updateController(controlDt);
  }

  if (millis() - lastPrintMs >= 200UL) {
    lastPrintMs = millis();
    printStatus();
  }

  delay(3);
}
