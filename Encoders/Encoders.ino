#define XOREncoderA = ?

volatile bool positionTable[2][4];
const int motorA = 0, motorB = 1;
const int aOld = 0, bOld = 1, aNew = 2, bNew = 3;

volatile bool A = 0, B = 0, XOR = 0;
int encoderACount = 0;


void setup() {
  // put your setup code here, to run once:
  //pinMode(EncoderAXOR, INPUT);
  attachInterrupt(digitalPintoInterrupt(XOREncoderA), EncoderAISR, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void EncoderAISR() {
  positionTable[0][0] = positionTable[0][2];
  positionTable[0][1] = positionTable[0][3];
  B = digitalRead(BpinEncoderA);
  XOR = digitalRead(ISRPinEncoderA);
  A = B ^ XOR;
  positionTable[0][2] = A;
  positionTable[0][3] = B;

  if ((positionTable[0][2] == HIGH && positionTable[0][1] == HIGH) || (positionTable[0][2] == LOW && positionTable[0][1] == LOW)) {
    encoderACount++;
  } 
  else if ((positionTable[0][2] == HIGH && positionTable[0][1] == LOW) || (positionTable[0][2] == LOW && positionTable[0][1] == HIGH)) {
    encoderACount--;
  }
  
  Serial.print("encoderACount = ");
  Serial.println(encoderACount);
}