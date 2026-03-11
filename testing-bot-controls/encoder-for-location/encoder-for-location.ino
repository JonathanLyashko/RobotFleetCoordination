#include <PRIZM.h>
#include <math.h>

PRIZM prizm;

// ==============================
// Robot geometry
// ==============================
const float WHEEL_DIAMETER_CM = 10.16;     // 4 inches
const float WHEEL_BASE_CM = 26.035;        // 10.25 inches
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;

// ==============================
// Pose state
// ==============================
float x_cm = 0.0;
float y_cm = 0.0;
float theta_rad = PI / 2;

long prevLeftDeg = 0;
long prevRightDeg = 0;

// ==============================
// Helpers
// ==============================
float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

int cmToMotorDegrees(float distance_cm) {
  return (int)round((distance_cm / WHEEL_CIRCUMFERENCE_CM) * 360.0);
}

int robotTurnDegToMotorDegrees(float robot_turn_deg) {
  float robot_turn_rad = robot_turn_deg * PI / 180.0;
  float wheel_travel_cm = (robot_turn_rad * WHEEL_BASE_CM) / 2.0;
  return cmToMotorDegrees(wheel_travel_cm);
}

void updatePoseFromEncoders(long leftDeg, long rightDeg) {
  long deltaLeftDeg = leftDeg - prevLeftDeg;
  long deltaRightDeg = rightDeg - prevRightDeg;

  prevLeftDeg = leftDeg;
  prevRightDeg = rightDeg;

  // Convert motor degrees to wheel travel in cm
  float dL = ((float)deltaLeftDeg / 360.0) * WHEEL_CIRCUMFERENCE_CM;
  float dR = ((float)deltaRightDeg / 360.0) * WHEEL_CIRCUMFERENCE_CM;

  // Given your earlier driveStraight uses motor 2 with negative degrees for forward,
  // this is probably needed:
  dL = -dL;

  float dCenter = (dL + dR) / 2.0;
  float dTheta = (dR - dL) / WHEEL_BASE_CM;

  float thetaMid = theta_rad + dTheta / 2.0;

  x_cm += dCenter * cos(thetaMid);
  y_cm += dCenter * sin(thetaMid);
  theta_rad = normalizeAngle(theta_rad + dTheta);
}

unsigned long lastSend = 0;
void waitForBothMotors() {
  while (prizm.readMotorBusy(1) == 1 || prizm.readMotorBusy(2) == 1) {

    long leftDeg  = prizm.readEncoderDegrees(2);
    long rightDeg = prizm.readEncoderDegrees(1);

    updatePoseFromEncoders(leftDeg, rightDeg);

    if (millis() - lastSend > 50) {
        printPoseJSON();
        lastSend = millis();
    }
  }
}

void printPoseJSON() {

  unsigned long t_ms = millis();

  float theta_deg = theta_rad * 180.0 / PI;

  int left_ultrasonic_cm = prizm.readSonicSensorCM(2);
  int front_ultrasonic_cm = prizm.readSonicSensorCM(4);

  Serial.print("{\"t_ms\":");
  Serial.print(t_ms);

  Serial.print(",\"x_cm\":");
  Serial.print(x_cm, 3);

  Serial.print(",\"y_cm\":");
  Serial.print(y_cm, 3);

  Serial.print(",\"theta_deg\":");
  Serial.print(theta_deg, 3);

  Serial.print(",\"front_ultrasonic_cm\":");
  Serial.print(front_ultrasonic_cm);

  Serial.print(",\"left_ultrasonic_cm\":");
  Serial.print(left_ultrasonic_cm);

  Serial.println("}");
}

// ==============================
// Motion primitives
// ==============================
void driveStraight(float speed_deg_per_sec, float distance_cm) {
  int motor_deg = cmToMotorDegrees(distance_cm);

  prizm.setMotorDegrees(speed_deg_per_sec, motor_deg,
                        speed_deg_per_sec, -motor_deg);

  waitForBothMotors();
  printPoseJSON();
}

void turnInPlace(float speed_deg_per_sec, float robot_turn_deg) {
  int motor_deg = robotTurnDegToMotorDegrees(fabs(robot_turn_deg));

  if (robot_turn_deg > 0) {
    prizm.setMotorDegrees(speed_deg_per_sec, motor_deg,
                          speed_deg_per_sec,  motor_deg);
  } else {
    prizm.setMotorDegrees(speed_deg_per_sec,  -motor_deg,
                          speed_deg_per_sec,  -motor_deg);
  }

  waitForBothMotors();
  printPoseJSON();
}

// ==============================
// Test routines
// ==============================
void testStraightOneRevolution() {
  prizm.resetEncoders();
  prevLeftDeg = 0;
  prevRightDeg = 0;
  driveStraight(200, WHEEL_CIRCUMFERENCE_CM);   // about 31.92 cm
  delay(1000);
}

void testTurnNinety() {
  prizm.resetEncoders();
  prevLeftDeg = 0;
  prevRightDeg = 0;
  turnInPlace(200, 90.0);
  delay(1000);
}

void testThreeSixty() {
  prizm.resetEncoders();
  prevLeftDeg = 0;
  prevRightDeg = 0;
  turnInPlace(100, 360.0);
  delay(1000);
}

void testSquarePath(float side_cm) {
  for (int i = 0; i < 4; i++) {

    prizm.resetEncoders();
    prevLeftDeg = 0;
    prevRightDeg = 0;
    driveStraight(200, side_cm);
    delay(500);

    prizm.resetEncoders();
    prevLeftDeg = 0;
    prevRightDeg = 0;
    turnInPlace(200, 90.0);
    delay(500);
  }
}

// ==============================
// Setup / loop
// ==============================
void setup() {
  prizm.PrizmBegin();
  Serial.begin(9600);
  printPoseJSON();

  driveStraight(200, WHEEL_CIRCUMFERENCE_CM);
  
  // testStraightOneRevolution();
  // testThreeSixty();
  // testSquarePath(20.0);
}

void loop() {
}