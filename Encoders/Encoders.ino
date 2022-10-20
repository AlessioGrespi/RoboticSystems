const int XOREncoderA = 26;
const int BpinEncoderA = PE2;
const int EncoderAINT = 26;  // --> IDK WTF IS GOING ON WITH THIS ENCODER INTERRUPT PIN I CANT WORK OUT WHAT IT IS

const int XOREncoderB = 7;
const int BpinEncoderB = 23;
const int EncoderBINT = 7;

volatile bool positionTable[2][4];
const int motorA = 0, motorB = 1;
const int aOld = 0, bOld = 1, aNew = 2, bNew = 3;

volatile bool A = 0, B = 0, XOR = 0;
int encoderACount = 0;
int encoderBCount = 0;

bool flag = false;


void setup() {
  // put your setup code here, to run once:

  //attachInterrupt(digitalPinToInterrupt(EncoderAINT), EncoderAISR, CHANGE);
  encoderASetup();
  attachInterrupt(digitalPinToInterrupt(EncoderBINT), EncoderBISR, CHANGE);

  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Setup Complete");
}

void loop() {
  // put your main code here, to run repeatedly:
}
void EncoderBISR() {
  positionTable[motorB][aOld] = positionTable[motorB][aNew];
  positionTable[motorB][bOld] = positionTable[motorB][bNew];
  B = digitalRead(BpinEncoderB);
  XOR = digitalRead(XOREncoderB);
  A = B ^ XOR;
  positionTable[motorB][aNew] = A;
  positionTable[motorB][bNew] = B;

  if ((positionTable[motorB][aNew] == HIGH && positionTable[motorB][bOld] == HIGH) || (positionTable[motorB][aNew] == LOW && positionTable[motorB][bOld] == LOW)) {
    encoderBCount++;
    Serial.print("Pos++ | ");
  } else if ((positionTable[motorB][aNew] == HIGH && positionTable[motorB][bOld] == LOW) || (positionTable[motorB][aNew] == LOW && positionTable[motorB][bOld] == HIGH)) {
    encoderBCount--;
    Serial.print("Neg-- | ");
  }

  Serial.print("EncoderBCount = ");
  Serial.print(encoderBCount);
  Serial.print(" | ");
  Serial.print(A);
  Serial.print(B);
  Serial.print(" | ");
  Serial.print(B);
  Serial.println(XOR);
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
    encoderACount++;
  } else if ((positionTable[motorA][aNew] == HIGH && positionTable[motorA][bOld] == LOW) || (positionTable[motorA][aNew] == LOW && positionTable[motorA][bOld] == HIGH)) {
    encoderACount--;
  }

  Serial.print("EncoderACount = ");
  Serial.print(encoderACount);
  Serial.print(" | ");
  Serial.print(A);
  Serial.print(B);
  Serial.print(" | ");
  Serial.print(B);
  Serial.println(XOR);
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
