// #include "motors.h"
//#include "Serial.h"

// Motor Variables

// https://www.pololu.com/docs/0J83/5.9
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD ?
#define REV ?

//functions
void motorMove();
void robotFullStop();
void irRead();

// LED Pins

// Light Sensors

#define L_LS_PIN A1
#define M_LS_PIN A2
#define R_LS_PIN A3
#define IR_LED_PWR_PIN 11
#define LS_ARRAY_ADDRESS[3] = {L_LS_PIN, M_LS_PIN, R_LS_PIN}

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
  digitalWrite(IR_LED_PWR_PIN, HIGH);  // switch on IR LEDs
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

void irRead() { // CONVERT TO READ AND WRITE LS ARRAY

  // Charge capacitor by setting input pin
  // temporarily to output and HIGH
  pinMode(LS_LEFT_PIN, OUTPUT);
  digitalWrite(LS_LEFT_PIN, HIGH);

  // Tiny delay for capacitor to charge.
  delayMicroseconds(10);

  //  Turn input pin back to an input
  pinMode(LS_LEFT_PIN, INPUT);


  // We are interested in the elapsed
  // time for the sensor read.
  unsigned long sensor_time;

  // Begin measurement of time in
  // microseconds
  unsigned long start_time;
  start_time = micros();

  // Use a flag to capture if the sensor
  // read is complete.
  // Set to false (not done) initially.
  bool done = false;

  // We want to stop the process if it takes
  // too long. Here, we will set a "time out"
  // value to compare against.
  unsigned long timeout = 5000;

  // We will use the general flag "done" to
  // decide if we should continue waiting.
  // ( "done is not true", so run while loop)

  while (done != true) {

    // Get current time
    unsigned long current_time = micros();

    // Determine elapsed time
    unsigned long elapsed_time = current_time - start_time;

    // Check if we have been waiting too long
    // If true, break from the loop.
    if (elapsed_time >= timeout) {

      done = true;
      sensor_time = timeout;
    }

    // If the pin has gone low, then the
    // read is complete.  Save elapsed time
    // and change done flag to true.
    if (digitalRead(LS_LEFT_PIN) == LOW) {

      sensor_time = elapsed_time;
      done = true;
    }

    // LS_LEFT_PIN must have gone LOW,
    // store microsecond count afterwards
    

    // Print output.
    Serial.print("Left line sensor: ");
    Serial.print(elapsed_time);
    Serial.print("\n");

    // Delay, just so that we have time toread the
    // values when using Serial.print().
    delay(100);
  }