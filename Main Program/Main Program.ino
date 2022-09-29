// #include "motors.h"
#include "Serial.h"

// Motor Variables

// https://www.pololu.com/docs/0J83/5.9
#define L_PWM_PIN ?
#define L_DIR_PIN ?
#define R_PWM_PIN ?
#define R_DIR_PIN ?

#define FWD ?
#define REV ?

//functions
void motorMove();
void robotFullStop();

// LED Pins

// Light Sensors

#define L_LS_PIN A1;
#define M_LS_PIN A2;
#define R_LS_PIN A3;
#define IR_LED_PWR_PIN 11;

// Steering Maths

// Path Logic Maths

void setup() {
  // Motor Setup
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  // Light Sensor Setup
  pinMode(IR_LED_PWR_PIN, OUTPUT);
  pinMode(L_LS_PIN, INPUT);
  pinMode(M_LS_PIN, INPUT);
  pinMode(R_LS_PIN, INPUT);
  Serial.begin(9600);
  digitalWrite(IR_LED_PWR_PIN, HIGH); // switch on IR LEDs
}

void loop() {
}

void motorMove(lSpeed, lDirection, rSpeed, rDirection) {
  analogWrite(L_PWM_PIN, lSpeed);
  digitalWrite(L_DIR_PIN, lDirection);
  analogWrite(R_PWM_PIN, rSpeed);
  digitalWrite(R_DIR_PIN, rDirection);
}

void robotFullStop() {
  motorMove(0, 0, 0, 0);
  while (1) {
    delay(500);
  }
}