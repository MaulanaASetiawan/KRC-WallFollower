#define SPEED 63
#define DIST_PROPORTIONAL_CONST 0.034/2
 
#include <Wire.h>
#include <LiquidCrystal.h>
 
//right motor pins
const int in1R = 4;
const int in2R = 5;
const int enR = 3;
 
//left motor pins
const int in1L = 8;
const int in2L = 9;
const int enL = 6;
 
//sensor 1 pins
const int trigLeft = A1;
const int echoLeft = A0;
 
//sensor 2 pins
const int trigRight = A5;
const int echoRight = A4;
 
//sensor 3 pins : front sensor
const int trigMiddle = A3;
const int echoMiddle = A2;

const int powerSwitch = 10;
 
int kiri, kanan, tengah, jkr, jkn, jdp;
 
void maju(){
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, HIGH);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
}
 
void putar(){
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, HIGH);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
}
 
void mundur(){
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
  delay(50);
}
 
void stop(){
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
}
 
void belokkiri(){
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, HIGH);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
  delay(100);
}
 
void belokagakkiri(){
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
  delay(200);
}
 
void belokkanan(){
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
  delay(100);
}
 
void belokagakkanan(){
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, HIGH);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  analogWrite(enL,SPEED);
  analogWrite(enR, SPEED);
  delay(200);
}
 
void setup() {
  Serial.begin(9600);
  pinMode(in1L,OUTPUT); // motor
  pinMode(in2L,OUTPUT); // motor
  pinMode(in1R,OUTPUT); // motor
  pinMode(in2R,OUTPUT); // motor
  pinMode(enL,OUTPUT); // motor
  pinMode(enR,OUTPUT); // motor
 
  pinMode(trigLeft,OUTPUT); // sensor
  pinMode(echoLeft,INPUT); // sensor
  pinMode(trigRight,OUTPUT); // sensor
  pinMode(echoRight,INPUT); // sensor
  pinMode(trigMiddle,OUTPUT); // sensor
  pinMode(echoMiddle,INPUT); // sensor

  pinMode(powerSwitch, INPUT_PULLUP); // Assuming the switch connects the pin to GND when pressed
}
 
void loop() {
  int switchState = digitalRead(powerSwitch);

  if (switchState == LOW) {
    digitalWrite(trigLeft, LOW);
    delayMicroseconds(2);
    digitalWrite(trigLeft, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigLeft, LOW);
    kiri = pulseIn(echoLeft, HIGH);
    jkr = kiri*0.0343/2;

    digitalWrite(trigRight, LOW);
    delayMicroseconds(2);
    digitalWrite(trigRight, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigRight, LOW);
    kanan = pulseIn(echoRight, HIGH);
    jkn = kanan*0.0343/2;

    digitalWrite(trigMiddle, LOW);
    delayMicroseconds(2);
    digitalWrite(trigMiddle, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigMiddle, LOW);
    tengah = pulseIn(echoMiddle, HIGH);
    jdp = tengah*0.0343/2;

    Serial.print("kiri : ");
    Serial.println(jkr);
    Serial.print("kanan : ");
    Serial.println(jkn);
    Serial.print("tengah : ");
    Serial.println(jdp); 
  
    
    if (jdp <= 10) {
      mundur();
      if(jkn<jkr){
        belokkiri();
      } else if (jkr<jkn){
        belokkanan();
      } else if (jdp <= 10 && jkn <= 5 && jkr <= 5){
        stop();
        delay(1000);
        belokkanan();
      }
    }
  
    else if (jkr <= 5){
      belokkanan();
    }
    else if (jkn <= 5){
      belokkiri();
    }
    else {
      maju();
    }
  } else {
    digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    analogWrite(enL,0);
    analogWrite(enR, 0);
  }
}