#include <Wire.h>
#include <math.h>

#ifndef MODE
#define MODE 0
#endif

//pins
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const int PWMA = 25, AIN1 = 26, AIN2 = 27;   // motor A
const int PWMB = 18, BIN1 = 32, BIN2 = 33;   // motor B
const int STBY = 4;

#ifndef FWD_AXIS
#define FWD_AXIS 0
#endif
const float IMU_SIGN = 1.0f;

//motors
const bool MOTOR_A_INV = false;   // flip direction of A
const bool MOTOR_B_INV = false;   // flip direction of B
const int MIN_PWM = 40;           // motor needs this much to move
const int MAX_PWM = 200;          // power cap
const int PWM_FREQ = 20000;       
const int PWM_BITS = 8;           

// control
const float ALPHA = 0.98f;                // filter, gyro vs accelerometer
const unsigned long LOOP_US = 5000;       
const float FALL_ANGLE = 35.0f;           
const float START_ANGLE = 3.0f;           
const unsigned long START_HOLD_MS = 500;  

float Kp = 12.0f, Ki = 0.0f, Kd = 0.6f;   // starting guesses
float trim = 0.0f;                        // angle (deg) where the bot actually balances

//IMU (raw I2C) 

const uint8_t MPU_ADDR = 0x68;   

struct Imu { float ax, ay, az, gx, gy, gz; };

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool initImu() {
  uint8_t who = 0;
  if (!mpuRead(0x75, &who, 1)) return false;
  Serial.printf("WHO_AM_I = 0x%02X\n", who);   // MPU6050 -> 0x68, MPU9250 -> 0x71 (verify in datasheet)
  mpuWrite(0x6B, 0x01);   // wake up, use gyro clock
  delay(100);
  mpuWrite(0x1A, 0x02);   // internal low-pass filter ~98 Hz
  mpuWrite(0x1B, 0x08);   // gyro  range +-500 deg/s  (65.5 LSB per deg/s)
  mpuWrite(0x1C, 0x00);   // accel range +-2 g        (16384 LSB per g)
  return true;
}

bool readImu(Imu &d) {
  uint8_t b[14];
  if (!mpuRead(0x3B, b, 14)) return false;
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  // b[6], b[7] = temperature (unused)
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);
  d.ax = ax / 16384.0f;  d.ay = ay / 16384.0f;  d.az = az / 16384.0f;
  d.gx = gx / 65.5f;     d.gy = gy / 65.5f;     d.gz = gz / 65.5f;
  return true;
}

// Tilt from gravity. Forward lean = positive angle.
float accelAngle(const Imu &d) {
  float fwd = (FWD_AXIS == 0) ? d.ax : d.ay;
  return atan2f(-fwd, d.az) * 57.29578f * IMU_SIGN;
}

// Tipping speed (deg/s) about the wheel axle. Forward tip = positive.
float gyroRate(const Imu &d) {
  float r = (FWD_AXIS == 0) ? d.gy : -d.gx;
  return r * IMU_SIGN;
}

// MOTORS 

void pwmSetup() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);  ledcAttachPin(PWMA, 0);
  ledcSetup(1, PWM_FREQ, PWM_BITS);  ledcAttachPin(PWMB, 1);
#endif
}

void pwmWrite(int motor, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motor == 0 ? PWMA : PWMB, duty);
#else
  ledcWrite(motor, duty);
#endif
}

// motor: 0 = A, 1 = B.  speed: -255..255, positive = bot rolls FORWARD.
void driveMotor(int motor, int speed) {
  bool inv = (motor == 0) ? MOTOR_A_INV : MOTOR_B_INV;
  if (inv) speed = -speed;
  int in1 = (motor == 0) ? AIN1 : BIN1;
  int in2 = (motor == 0) ? AIN2 : BIN2;
  digitalWrite(in1, speed > 0 ? HIGH : LOW);
  digitalWrite(in2, speed < 0 ? HIGH : LOW);
  pwmWrite(motor, abs(speed));
}

void motorsOn()  { digitalWrite(STBY, HIGH); }

void motorsOff() {
  driveMotor(0, 0);
  driveMotor(1, 0);
  digitalWrite(STBY, LOW);
}

// STATE

float angle = 0.0f;      // filtered tilt (deg)
float rate = 0.0f;       // tipping speed (deg/s)
float gyroBias = 0.0f;
float integ = 0.0f;
bool active = false;
unsigned long upSince = 0;
unsigned long lastUs = 0;
unsigned long lastPrint = 0;

//SERIAL TUNING

void parseCommand(char *s) {
  float v = atof(s + 1);
  switch (s[0]) {
    case 'p': Kp = v; break;
    case 'i': Ki = v; integ = 0; break;
    case 'd': Kd = v; break;
    case 't': trim = v; break;
    default: break;
  }
  Serial.printf("Kp=%.2f Ki=%.3f Kd=%.3f trim=%.2f\n", Kp, Ki, Kd, trim);
}

void handleSerial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; parseCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

//SETUP

void calibrateGyro() {
  Serial.println("Calibrating gyro - keep the bot STILL...");
  float sum = 0.0f;
  int n = 0;
  for (int i = 0; i < 500; i++) {
    Imu d;
    if (readImu(d)) { sum += gyroRate(d); n++; }
    delay(3);
  }
  gyroBias = (n > 0) ? sum / n : 0.0f;
  Serial.printf("Gyro bias = %.3f deg/s\n", gyroBias);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pwmSetup();
  motorsOff();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!initImu()) {
    Serial.println("IMU not found. Check SDA/SCL/3V3/GND and AD0 -> GND.");
    while (true) delay(1000);   // stop here, motors stay off
  }

  calibrateGyro();

  Imu d;
  if (readImu(d)) angle = accelAngle(d);   // start the filter from the accelerometer

  upSince = millis();
  lastUs = micros();
  Serial.printf("Ready. MODE = %d\n", MODE);
}

//MOTOR TEST (MODE 1)

void motorTest() {
  motorsOn();
  Serial.println("Motor A forward");  driveMotor(0, 120); driveMotor(1, 0);   delay(1500);
  Serial.println("Motor A backward"); driveMotor(0, -120); driveMotor(1, 0);  delay(1500);
  Serial.println("Motor B forward");  driveMotor(0, 0); driveMotor(1, 120);   delay(1500);
  Serial.println("Motor B backward"); driveMotor(0, 0); driveMotor(1, -120);  delay(1500);
  Serial.println("Both forward (both wheels should roll the bot FORWARD)");
  driveMotor(0, 120); driveMotor(1, 120);                                     delay(2000);
  motorsOff();
  Serial.println("Stopped. Pausing...");
  delay(3000);
}

//BALANCE (MODE 2)

void balanceStep(float dt) {
  float err = angle - trim;

  if (!active) {
    // wait until the bot is held upright and still
    if (fabsf(err) < START_ANGLE) {
      if (millis() - upSince > START_HOLD_MS) {
        active = true;
        integ = 0.0f;
        motorsOn();
      }
    } else {
      upSince = millis();
    }
    return;
  }

  if (fabsf(angle) > FALL_ANGLE) {   // fell over
    active = false;
    motorsOff();
    upSince = millis();
    return;
  }

  integ += err * dt;
  integ = constrain(integ, -50.0f, 50.0f);

  float u = Kp * err + Ki * integ + Kd * rate;

  int pwm = 0;
  float a = fabsf(u);
  if (a > 1.0f) pwm = (int)fminf(a + MIN_PWM, (float)MAX_PWM);
  int speed = (u > 0) ? pwm : -pwm;

  driveMotor(0, speed);
  driveMotor(1, speed);
}

//MAIN LOOP

void loop() {
#if MODE == 1
  motorTest();
  return;
#endif

  handleSerial();

  unsigned long now = micros();
  if (now - lastUs < LOOP_US) return;
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;

  Imu d;
  if (!readImu(d)) {
    motorsOff();
    active = false;
    return;
  }

  // ---- 1. SENSE: one clean angle ----
  rate = gyroRate(d) - gyroBias;
  float accAng = accelAngle(d);
  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * accAng;

  // ---- 2. THINK + 3. ACT ----
#if MODE == 2
  balanceStep(dt);
#endif

  // ---- print at 10 Hz (printing every loop would slow the 200 Hz timing) ----
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("angle:%.1f rate:%.1f active:%d\n", angle, rate, active ? 1 : 0);
  }
}
