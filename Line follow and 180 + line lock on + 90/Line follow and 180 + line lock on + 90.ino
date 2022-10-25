#include <string.h>

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define LL_LS_PIN A11
#define L_LS_PIN A0
#define M_LS_PIN A2
#define R_LS_PIN A3
#define RR_LS_PIN A4
#define IR_LED_PWR_PIN 11

unsigned long elapsed_time[5];

char instruction[10];
int direction, case2Start = 0;

float wleft = 0, wright = 0, eline = 0, controlSignal = 0;
int mleft = 0, mright = 0;

unsigned long LSthreshold = 1500;

int FSM = 1;

bool timerFlag = false;


int lineDetected;

void setup() {
  // put your setup code here, to run once:
  // Motor Setup
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  pinMode(IR_LED_PWR_PIN, OUTPUT);

  pinMode(LL_LS_PIN, INPUT);
  pinMode(L_LS_PIN, INPUT);
  pinMode(M_LS_PIN, INPUT);
  pinMode(R_LS_PIN, INPUT);
  pinMode(RR_LS_PIN, INPUT);

  digitalWrite(IR_LED_PWR_PIN, HIGH);  // switch on IR LEDs

  Serial.begin(9600);

  // while (!Serial)
  //   ;  //comment out when not connected to laptop
  Serial.print("Setup Complete \n");

  //motorMove(50, 0, 50, 0);
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  int SensorState = 0;
  irRead();

  switch (FSM) {
    case 1:
      motorMove(40, 40);
      if ((elapsed_time[1] > 1000) || (elapsed_time[2] > 1000) || (elapsed_time[3] > 1000)) {
        delay(200);
        motorMove(0, 0);
        irRead();
        if ((elapsed_time[0] > 1000) || (elapsed_time[1] > 1000)) {
          while (elapsed_time[2] < 1200) {
            irRead();
            motorMove(40, -40);
          }
        } else if ((elapsed_time[3] > 1000) || (elapsed_time[4] > 1000)) {
          while (elapsed_time[2] < 1200) {
            irRead();
            motorMove(-40, 40);
          }
        } else {
          while (elapsed_time[2] < 1200) {
            irRead();
            motorMove(-40, 40);
          }
        }
        motorMove(0, 0);
        FSM = 2;
      }
      break;
    case 2:
      if (timerFlag == false) {
        case2Start = millis() + 10000;
        timerFlag = true;
      }

      dumbController();

      //if (millis() < case2Start) { //180
      if ((elapsed_time[0] < 1000) && (elapsed_time[1] < 1000) && (elapsed_time[2] < 1000) && (elapsed_time[3] < 1000) && (elapsed_time[4] < 1000)) {
        motorMove(0, 0);
        delay(1000);
        motorMove(40, -40);
        delay(690);
        motorMove(0, 0);
      }
      //}

      //90
      if ((elapsed_time[0] > 1200) || (elapsed_time[4] > 1200)) {
        delay(200);
        motorMove(0, 0);
        delay(200);
        if (elapsed_time[4] > 1200) {
          do {
            Serial.print("stuck | ");
            irRead();
            motorMove(30, -30);
          } while (elapsed_time[2] < 1200);
        }
        if (elapsed_time[0] > 1200) {
          do {
            Serial.print("stuck | ");
            irRead();
            motorMove(-30, 30);
          } while (elapsed_time[2] < 1200);
        }
      }
      break;
    case 3:
      motorMove(40, 40);
      if ((elapsed_time[1] > 1000) || (elapsed_time[2] > 1000) || (elapsed_time[3] > 1000)) {
        FSM = 4;
      }
      break;
  }


  // Serial.print(" | ");
  // Serial.print(wleft);
  // Serial.print(" | ");
  // Serial.print(wright);
  // // Serial.print(" | ");
  // // Serial.print(mleft);
  // // Serial.print(" | ");
  // // Serial.print(mright);
  // Serial.print(" | ");
  // Serial.print(eline);
  // Serial.print(FSM);
  // Serial.print(" | ");
  // Serial.print(case2Start);
  // Serial.print(" | ");
  // Serial.print(controlSignal);

  //Serial.println("");
}


void dumbController() {

  float LLS, MLS, RLS, total;


  LLS = elapsed_time[1];
  MLS = elapsed_time[2];
  RLS = elapsed_time[3];

  total = LLS + MLS + RLS;
  LLS /= total;
  MLS /= total;
  RLS /= total;

  wleft = LLS + (MLS * 0.5);
  wright = RLS + (MLS * 0.5);

  eline = wleft - wright;

  float Kp = 1;
  controlSignal = eline * Kp;

  float maxTurn = 50, forwardBias = 40;



  maxTurn *= controlSignal;

  motorMove(forwardBias - maxTurn, forwardBias + maxTurn);
}


void motorMove(int lPWM, int rPWM) {

  if (lPWM > 0) {
    digitalWrite(L_DIR_PIN, 0);

  } else if (lPWM < 0) {
    digitalWrite(L_DIR_PIN, 1);
    lPWM = abs(lPWM);
  }
  if (rPWM > 0) {
    digitalWrite(R_DIR_PIN, 0);

  } else if (rPWM < 0) {
    digitalWrite(R_DIR_PIN, 1);
    rPWM = abs(rPWM);
  }

  analogWrite(L_PWM_PIN, lPWM);
  analogWrite(R_PWM_PIN, rPWM);
}

void irRead() {

  int pinCheckSum = 0;
  int block[5];
  unsigned long end_time[5];
  unsigned long start_time;

  for (int m = 0; m < 5; m++) {
    block[m] = 0;
  }

  int LS_ARRAY_ADDRESS[5] = { LL_LS_PIN, L_LS_PIN, M_LS_PIN, R_LS_PIN, RR_LS_PIN };

  for (int j = 0; j < 5; j++) {
    pinMode(LS_ARRAY_ADDRESS[j], OUTPUT);
    digitalWrite(LS_ARRAY_ADDRESS[j], HIGH);
  }

  delayMicroseconds(10);

  for (int k = 0; k < 5; k++) {
    pinMode(LS_ARRAY_ADDRESS[k], INPUT);
  }

  start_time = micros();

  while (pinCheckSum < 5) {
    for (int l = 0; l < 5; l++) {
      if (digitalRead(LS_ARRAY_ADDRESS[l]) != HIGH && (block[l] == 0)) {
        pinCheckSum = pinCheckSum + 1;
        end_time[l] = micros();
        block[l] = 1;
      }
    }
  }

  lineDetected = 4;

  for (int i = 0; i < 5; i++) {
    elapsed_time[i] = end_time[i] - start_time;
    Serial.print(elapsed_time[i]);
    Serial.print(" | ");
    // if (i < 4) {
    //   Serial.print(" | ");
    // }
    if (elapsed_time[i] > LSthreshold) {
      lineDetected++;
    } else if (elapsed_time[i] < LSthreshold) {
      lineDetected--;
    }
    //delay(100);
  }

  // Serial.print(lineDetected);
  // Serial.print(" | ");
  // if (lineDetected > 0) {
  //   Serial.print("Line Detected");
  // } else {
  //   Serial.print("No Line Detected");
  // }
  Serial.println("");
  lineDetected = 4;
}