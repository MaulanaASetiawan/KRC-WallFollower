#include <NewPing.h>

#define rightUltrasonicTrig A5
#define rightUltrasonicEcho A4

#define middleUltrasonicTrig A3
#define middleUltrasonicEcho A2

#define leftUltrasonicTrig A1
#define leftUltrasonicEcho A0

#define ENA 3
#define ENB 6
#define IN1 8
#define IN2 9
#define IN3 4
#define IN4 5

#define CONSTANT_SPEED_FORWARD 70
#define CONSTANT_SPEED_BACKWARD 50

#define SETPOINT 50
double Kp = 5.0; // Proportional gain
double Ki = 0.1; // Integral gain
double Kd = 0.2; // Derivative gain

double prevError = 0;
double integral = 0;

NewPing sonarLeft(leftUltrasonicTrig, leftUltrasonicEcho, 200);
NewPing sonarMiddle(middleUltrasonicTrig, middleUltrasonicEcho, 200);
NewPing sonarRight(rightUltrasonicTrig, rightUltrasonicEcho, 200);

int readDistance(NewPing& sonar) {
  delay(50);
  unsigned int uS = sonar.ping();
  int distance = uS / US_ROUNDTRIP_CM;
  return distance;
}

double calculatePID(double currentDistance) {
  double error = SETPOINT - currentDistance;
  integral += error;
  double derivative = error - prevError;

  double output = Kp * error + Ki * integral + Kd * derivative;

  prevError = error;
  return output;
}

void setup() {
  Serial.begin(115200);

  pinMode(rightUltrasonicEcho, INPUT);
  pinMode(rightUltrasonicTrig, OUTPUT);

  pinMode(middleUltrasonicEcho, INPUT);
  pinMode(middleUltrasonicTrig, OUTPUT);

  pinMode(leftUltrasonicEcho, INPUT);
  pinMode(leftUltrasonicTrig, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  double distanceLeft = readDistance(sonarLeft);
  double distanceMiddle = readDistance(sonarMiddle);
  double distanceRight = readDistance(sonarRight);

  double outputLeft = calculatePID(distanceLeft);
  double outputMiddle = calculatePID(distanceMiddle);
  double outputRight = calculatePID(distanceRight);

  // Serial.print("PID Left: ");
  // Serial.println(outputLeft);
  // Serial.print("PID Middle: ");
  // Serial.println(outputMiddle);
  // Serial.print("PID Right: ");
  // Serial.println(outputRight);

  double error = distanceMiddle - 15;  // Set the desired distance from the wall
  double output = calculatePID(error);

  int leftSpeed = CONSTANT_SPEED_FORWARD - output;
  int rightSpeed = CONSTANT_SPEED_FORWARD + output;
  Serial.print("Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Center: ");
  Serial.print(distanceMiddle);
  Serial.print(" cm, Right: ");
  Serial.print(distanceRight);
  Serial.print(" cm, Output: ");
  Serial.println(output);
}
