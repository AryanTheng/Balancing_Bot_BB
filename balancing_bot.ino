#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <L298NX2.h>
#include <PID_v1.h>

/* ---------------- MPU6050 ---------------- */
Adafruit_MPU6050 mpu;

/* ---------------- MOTOR PINS ---------------- */
// Motor A (Left)
const unsigned int EN_A  = 5;
const unsigned int IN1_A = 4;
const unsigned int IN2_A = 2;

// Motor B (Right)
const unsigned int EN_B  = 6;
const unsigned int IN1_B = 8;
const unsigned int IN2_B = 7;

// Initialize motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

/* ---------------- PID VARIABLES ---------------- */
double Setpoint;
double Input;
double Output;

// PID tuning
double Kp = 20.0;
double Ki = 0.5;
double Kd = 1.2;

PID balancePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/* ---------------- MOTOR SETTINGS ---------------- */
const int MIN_PWM_A = 30;
const int MIN_PWM_B = 30;
const int MAX_PWM   = 255;

float deadband = 0.2;

/* ---------------- SAFETY LIMITS ---------------- */
const float MAX_ANGLE = 50.0;
const float MIN_ANGLE = -50.0;

bool botEnabled = true;

/* ---------------- FILTER VARIABLES ---------------- */
float angle = 0.0;
float gyroRate = 0.0;
unsigned long lastTime;

/* ---------------- CALIBRATION ---------------- */
const unsigned long CALIB_TIME_MS = 10000;

/* ---------------- SETUP ---------------- */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  motors.stopA();
  motors.stopB();

  Serial.println("Hold bot upright...");
  Serial.println("Calibrating target angle (10s)");

  /* -------- AUTO CALIBRATION -------- */
  unsigned long startTime = millis();
  double angleSum = 0;
  int samples = 0;

  while (millis() - startTime < CALIB_TIME_MS) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    double accAngle =
      -atan2(a.acceleration.x, a.acceleration.z) * 57.2958;

    angleSum += accAngle;
    samples++;
    delay(5);
  }

  Setpoint = angleSum / samples;
  angle = Setpoint;

  Serial.print("Target Angle = ");
  Serial.println(Setpoint, 3);

  /* -------- PID SETTINGS -------- */
  balancePID.SetMode(AUTOMATIC);
  balancePID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  balancePID.SetSampleTime(10);   // slower = less noise reaction

  lastTime = millis();
  Serial.println("Balancing STARTED");
}

/* ---------------- LOOP ---------------- */
void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  /* -------- TIME STEP -------- */
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  /* -------- SENSOR FUSION -------- */
  float accAngle =
    -atan2(a.acceleration.x, a.acceleration.z) * 57.2958;

  gyroRate = g.gyro.y * 57.2958;  // deg/sec

  // Complementary filter
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;

  Input = angle;

  /* -------- SAFETY CHECK -------- */
  if (Input > MAX_ANGLE || Input < MIN_ANGLE) {
    motors.stopA();
    motors.stopB();
    botEnabled = false;
    Serial.println("Angle out of range! Motors stopped.");
    delay(5);
    return;
  }

  if (!botEnabled && Input >= MIN_ANGLE && Input <= MAX_ANGLE) {
    botEnabled = true;
    Serial.println("Angle safe. Resuming balance.");
  }

  if (!botEnabled) return;

  /* -------- DEAD BAND -------- */
  if (abs(Input - Setpoint) < deadband) {
    motors.stopA();
    motors.stopB();
    return;
  }

  /* -------- PID COMPUTE -------- */
  balancePID.Compute();

  /* -------- SOFT MOTOR DRIVE -------- */
  int pwmBase = constrain(abs((int)Output), 0, MAX_PWM);

  int pwmA = constrain(pwmBase + MIN_PWM_A, MIN_PWM_A, MAX_PWM);
  int pwmB = constrain(pwmBase + MIN_PWM_B, MIN_PWM_B, MAX_PWM);

  if (Output > 0) {
    motors.setSpeedA(pwmA);
    motors.setSpeedB(pwmB);
    motors.forwardA();
    motors.forwardB();
  } else {
    motors.setSpeedA(pwmA);
    motors.setSpeedB(pwmB);
    motors.backwardA();
    motors.backwardB();
  }

  /* -------- DEBUG -------- */
  Serial.print("Angle: ");
  Serial.print(Input, 2);
  Serial.print(" | Acc: ");
  Serial.print(accAngle, 2);
  Serial.print(" | Output: ");
  Serial.print(Output);
  Serial.print(" | PWM: ");
  Serial.println(pwmA);

  delay(5);
}