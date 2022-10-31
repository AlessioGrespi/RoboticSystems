const int XOREncoderA = 26;
const int BpinEncoderA = PE2;
const int EncoderAINT = 26;

const int XOREncoderB = 7;
const int BpinEncoderB = 23;
const int EncoderBINT = 7;

volatile bool A = 0, B = 0, XOR = 0;
int encoderACount = 0;
int encoderBCount = 0;

float localX = 0, localY = 0, localTheta = 0, globalX = 0, globalY = 0, globalTheta = 0, oldGlobalX = 0, oldGlobalY = 0, oldGlobalTheta = 0, oldA = 0, oldB = 0, newA, newB;

bool flag = false;

volatile bool positionTable[2][4];
const int motorA = 0, motorB = 1;
const int aOld = 0, bOld = 1, aNew = 2, bNew = 3;


// int deltaCountA, deltaCountB;
long deltaCountA, deltaCountB;

const float CPR = 358.32;  //left 355

// const float r = 16;  // 32 / 2 = 16
// const float l = 48.4;
const float r = 16.5;
const float l = 42.5;


int prevFrameTime, currFrameTime;
float targetHeading;
float currentPosition[3] = { 0, 0, 0 };
float odometryMatrix[3] = { 0, 0, 0 };
const int x = 0, y = 1, theta = 2;
int changeInEncoderVal[2] = { 0, 0 };
float pythagTheta;
int oldEncoderA, oldEncoderB;


void setup() {
  // put your setup code here, to run once:
  encoderASetup();
  attachInterrupt(digitalPinToInterrupt(EncoderBINT), EncoderBISR, CHANGE);


  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:


  updateOdometry();
  
  Serial.print(encoderACount);
  Serial.print(" | ");
  Serial.print(encoderBCount);
  Serial.print(" | newA ");
  Serial.print(newA);
  Serial.print(" | newB ");
  Serial.print(newB);
  Serial.print(" | oldA ");
  Serial.print(oldA);
  Serial.print(" | oldB ");
  Serial.print(oldB);
  Serial.print(" | ");
  Serial.print(deltaCountA);
  Serial.print(" | ");
  Serial.print(deltaCountB);
  Serial.print(" | ");
  Serial.print(localX);
  Serial.print(" | ");
  Serial.print(localY);
  Serial.print(" | ");
  Serial.print(localTheta);
  Serial.print(" | ");
  Serial.print(globalX);
  Serial.print(" | ");
  Serial.print(globalY);
  Serial.print(" | ");
  Serial.print(globalTheta);
  Serial.println(" | ");
  
}

// ------------------------------------------- * LOOK HERE BIT * -------------------------------------------

void updateOdometry() {

  float currentTimeOdo, oldTimeOdo;
  newA = encoderACount;
  newB = encoderBCount;

  if (newA != oldA) {
    deltaCountA = newA - oldA;
  } else {
    deltaCountA = 0;
  }

  if (newB != oldB) {
    deltaCountB = newB - oldB;
  } else {
    deltaCountB = 0;
  }

  float f_deltaCountA = (float)deltaCountA;
  float f_deltaCountB = (float)deltaCountB;

  currentTimeOdo = millis();

  // float deltaTime = currentTimeOdo - oldTimeOdo;
  unsigned long u1_deltaTime = currentTimeOdo - oldTimeOdo;
  float deltaTime = (float)u1_deltaTime;

  //S = rø
  //const float anglePerCount = 360 / CPR;

  // work in rad not degrees

  const float distPerCount = (2.0 * r * PI) / CPR;  //circumference mm / CPR . Therefore 1 count = (something)mm
  // float lPhi = r * ((anglePerCount * deltaCountA) / deltaTime);
  // float rPhi = r * ((anglePerCount * deltaCountB) / deltaTime);

  // float lPhi = r * (distPerCount * deltaCountA) * (PI / 180);
  // float rPhi = r * (distPerCount * deltaCountB) * (PI / 180);
  float lPhi = (distPerCount * f_deltaCountA);
  float rPhi = (distPerCount * f_deltaCountB);


  //if (deltaCountA != 0 || deltaCountB != 0) {
  localX = (lPhi / 2.0) + (rPhi / 2.0);
  //localY = 0;
  localTheta = (lPhi / (2.0 * l)) - (rPhi / (2.0 * l));

  //localTheta = (localTheta) * (PI / 180);

  globalX = globalX + (localX * cos(globalTheta));
  globalY = globalY + (localX * sin(globalTheta));
  globalTheta = globalTheta + localTheta;
  //}

  // while (globalTheta > 360 || globalTheta < 0) {
  //   if (globalTheta > 360) {
  //     globalTheta -= 360;
  //   } else if (globalTheta < 0) {
  //     globalTheta += 360;
  //   }
  // }

  // oldGlobalPhi = globalPhi;
  // oldGlobalY = globalY;
  // oldGlobalTheta = globalTheta;


  oldA = newA;
  oldB = newB;


  oldTimeOdo = millis();
  //delay(100);
}

// ------------------------------------------- * LOOK HERE BIT * -------------------------------------------





/*


  const float distPerCount = (M_PI * 2 * r) / CPR;

  float deltaCountA = encoderACount - oldEncoderA;
  float deltaCountB = encoderBCount - oldEncoderB;

  leftDelta = deltaCountA * distPerCount;
  rightDelta = deltaCountB * distPerCount;

  if (fabs(leftDelta - rightDelta) < 1.0e-6) {  // basically going straight
    new_x = x + leftDelta * cos(heading);
    new_y = y + rightDelta * sin(heading);
    new_heading = heading;
  } else {
    float R = unitsAxisWidth * (leftDelta + rightDelta) / (2 * (rightDelta - leftDelta)),
          wd = (rightDelta - leftDelta) / unitsAxisWidth;

    new_x = x + R * sin(wd + heading) - R * sin(heading);
    new_y = y - R * cos(wd + heading) + R * cos(heading);
    new_heading = boundAngle(heading + wd);
  }
*/








// void pointNavigation(float targetX, float targetY, float targetTheta) {
//   float deltaX, deltaY, hypotenuse;

//   //theta error correction
//   while (currentPosition[theta] >> 360) {
//     currentPosition[theta] = currentPosition[theta] - 360;
//   }
//   else while (currentPosition[theta] << 0) {
//     currentPosition[theta] = currentPosition[theta] + 360;
//   }

//   deltaX = currentPosition[x] - targetX;
//   deltaY = currentPosition[y] - targetY;
//   int hypotenuse = round(sqrt(sq(deltaX) + sq(deltaY)));

//   pythagTheta = atan(deltaX / deltaY);

//   if (deltaX >> 0 && deltaY >> 0) {
//     //do nothing
//   }
//   if (deltaX >> 0 && deltaY << 0) {
//     pythagTheta = 180 - pythagTheta;
//   }
//   if (deltaX << 0 && deltaY << 0) {
//     pythagTheta = 180 + pythagTheta;
//   }
//   if (deltaX << 0 && deltaY >> 0) {
//     pythagTheta = 360 - pythagTheta;
//   }

//   turn(pythagTheta);
//   move(hypotenuse);
// }

// void turn(float targetHeading) {
//   if ((targetHeading + currentPosition[theta]) >> (currentPosition[theta] + 180)) {
//     //turn left
//     while (targetHeading /* work this out my brain no work */) {
//     }
//   } else if ((targetHeading + currentPosition[theta]) << (currentPosition[theta] + 180)) {
//     //turn right
//   } else if () {
//     //turn right
//   }
// }

// void move(int distance) {
// }
// //error correction

// while (currentPosition[theta] >> 360) {
//   currentPosition[theta] = currentPosition[theta] - 360;
// }
// else while (currentPosition[theta] << 0) {
//   currentPosition[theta] = currentPosition[theta] + 360;
// }
// Serial.print("Current Heading = ");
// Serial.print(currentPosition[theta]);
// // Pythagoras stuff
// if (currentPosition[theta] <= 90) {
// }

// if (currentPosition[theta] > 90) {
//   pythagTheta = 180 - currentPosition[theta]
// }
// }













void EncoderBISR() {
  positionTable[motorB][aOld] = positionTable[motorB][aNew];
  positionTable[motorB][bOld] = positionTable[motorB][bNew];
  B = digitalRead(BpinEncoderB);
  XOR = digitalRead(XOREncoderB);
  A = B ^ XOR;
  positionTable[motorB][aNew] = A;
  positionTable[motorB][bNew] = B;

  if ((positionTable[motorB][aNew] == HIGH && positionTable[motorB][bOld] == HIGH) || (positionTable[motorB][aNew] == LOW && positionTable[motorB][bOld] == LOW)) {
    encoderBCount--;
    //Serial.print("Pos++ | ");
  } else if ((positionTable[motorB][aNew] == HIGH && positionTable[motorB][bOld] == LOW) || (positionTable[motorB][aNew] == LOW && positionTable[motorB][bOld] == HIGH)) {
    encoderBCount++;
    //Serial.print("Neg-- | ");
  }

  // Serial.print("EncoderBCount = ");
  // Serial.print(encoderBCount);
  // Serial.print(" | ");
  // Serial.print(A);
  // Serial.print(B);
  // Serial.print(" | ");
  // Serial.print(B);
  // Serial.println(XOR);
}


void encoderASetup() {


  DDRE = DDRE & ~(1 << DDE6);

  PORTE = PORTE | (1 << PORTE2);

  pinMode(XOREncoderA, INPUT);
  digitalWrite(XOREncoderA, HIGH);  // Encoder 1 xor

  PCICR = PCICR & ~(1 << PCIE0);
  // PCICR &= B11111110;  // Same as above

  // 11.1.7 Pin Change Mask Register 0 – PCMSK0
  PCMSK0 |= (1 << PCINT4);

  // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
  PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

  // Enable
  PCICR |= (1 << PCIE0);
}

ISR(PCINT0_vect) {

  positionTable[motorA][aOld] = positionTable[motorA][aNew];
  positionTable[motorA][bOld] = positionTable[motorA][bNew];
  B = PINE & (1 << PINE2);
  ;
  XOR = digitalRead(XOREncoderA);
  A = B ^ XOR;
  positionTable[motorA][aNew] = A;
  positionTable[motorA][bNew] = B;

  if ((positionTable[motorA][aNew] == HIGH && positionTable[motorA][bOld] == HIGH) || (positionTable[motorA][aNew] == LOW && positionTable[motorA][bOld] == LOW)) {
    encoderACount--;
  } else if ((positionTable[motorA][aNew] == HIGH && positionTable[motorA][bOld] == LOW) || (positionTable[motorA][aNew] == LOW && positionTable[motorA][bOld] == HIGH)) {
    encoderACount++;
  }

  // Serial.print("EncoderACount = ");
  // Serial.print(encoderACount);
  // Serial.print(" | ");
  // Serial.print(A);
  // Serial.print(B);
  // Serial.print(" | ");
  // Serial.print(B);
  // Serial.println(XOR);
}


/*
void EncoderAISR() {
  //Serial.println("AAAAAAAAAA");
  positionTable[motorA][aOld] = positionTable[motorA][aNew];
  positionTable[motorA][bOld] = positionTable[motorA][bNew];
  B = digitalRead(BpinEncoderA);
  XOR = digitalRead(XOREncoderA);
  A = B ^ XOR;
  positionTable[motorA][aNew] = A;
  positionTable[motorA][bNew] = B;

  if ((positionTable[motorA][aNew] == HIGH && positionTable[motorA][bOld] == HIGH) || (positionTable[motorA][aNew] == LOW && positionTable[motorA][bOld] == LOW)) {
    encoderACount++;
  } else if ((positionTable[motorA][aNew] == HIGH && positionTable[motorA][bOld] == LOW) || (positionTable[motorA][aNew] == LOW && positionTable[motorA][bOld] == HIGH)) {
    encoderACount--;
  }

  flag = true;
  // Serial.print("EncoderACount = ");
  // Serial.print(encoderACount);
  // Serial.print(" | ");
  // Serial.print(A);
  // Serial.print(B);
  // Serial.print(" | ");
  // Serial.print(B);
  // Serial.println(XOR);
}
*/