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

long durationRight;
long durationMiddle;
long durationLeft;

int distanceRight;
int distanceMiddle;
int distanceLeft;
int irSensorValue;

int safeDistance = 15; // Adjust this value based on your robot and environment

int getDistance(int trigPin, int echoPin) {
  // Function to measure distance using an ultrasonic sensor

  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse to the trigger pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin, LOW);

  // Measure the pulse duration on the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance using the speed of sound
  int distance = duration * 0.034 / 2;

  return distance;
}

void motorControl(bool leftDir, int leftSpeed, bool rightDir, int rightSpeed) {
  // Control both motors independently
  digitalWrite(IN1, !leftDir);
  digitalWrite(IN2, leftDir);
  analogWrite(ENA, leftSpeed);

  digitalWrite(IN3, rightDir);
  digitalWrite(IN4, !rightDir);
  analogWrite(ENB, rightSpeed);
}

void moveForward() {
  // If the middle sensor has a clear path, move forward
  motorControl(true, CONSTANT_SPEED_FORWARD, true, CONSTANT_SPEED_FORWARD);
}

void moveBackward() {
  // If the middle sensor has a clear path, move forward
  motorControl(false, CONSTANT_SPEED_BACKWARD, false, CONSTANT_SPEED_BACKWARD);
}

void turnLeft() {
  // Adjust left to follow the wall
  motorControl(true, CONSTANT_SPEED_FORWARD, true, 0);
  delay(100);
}

void turnRight() {
  // Adjust right to follow the wall
  motorControl(true, 0, true, CONSTANT_SPEED_FORWARD);
  delay(100);
}

void stop(){
  motorControl(true, 0, true, 0);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(rightUltrasonicTrig, OUTPUT);
  pinMode(middleUltrasonicTrig, OUTPUT);
  pinMode(leftUltrasonicTrig, OUTPUT);
  pinMode(rightUltrasonicEcho, INPUT);
  pinMode(middleUltrasonicEcho, INPUT);
  pinMode(leftUltrasonicEcho, INPUT);
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Measure distance for sensors
  distanceRight = getDistance(rightUltrasonicTrig, rightUltrasonicEcho);
  Serial.print("Distance Right: ");
  Serial.println(distanceRight);

  distanceMiddle = getDistance(middleUltrasonicTrig, middleUltrasonicEcho);
  Serial.print("Distance Middle: ");
  Serial.println(distanceMiddle);

  distanceLeft = getDistance(leftUltrasonicTrig, leftUltrasonicEcho);
  Serial.print("Distance Left: ");
  Serial.println(distanceLeft);


  if (distanceMiddle < 5 ) {
    stop();
    delay(100);
    moveBackward();
    delay(750);
  } else if (distanceMiddle <= safeDistance && (distanceLeft <= 15 || distanceRight <= 15)) {
    stop();
    if (distanceMiddle <= 15){
      if (distanceMiddle <= 5 && distanceLeft <= distanceRight) {
        moveBackward();
        delay(750);
        turnRight();
      } else if (distanceMiddle <= 5 && distanceRight <= distanceLeft) {
        moveBackward();
        delay(750);
        turnLeft();
      }
    }
  } else if (distanceLeft <= 15) {
    turnRight();
  } else if (distanceRight <= 15) {
    turnLeft();
  } else {
    moveForward();
  }

  // // Add delay for stability and to avoid rapid changes in motor speed
  delay(100);
}