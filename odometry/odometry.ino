const float r = ;
const float l = ;
int prevFrameTime, currFrameTime;
float currentPosition[3] = { 0, 0, 0 };
float odometryMatrix[3] = { 0, 0, 0 };
int changeInEncoderVal[2] = { 0, 0 };
float pythagTheta;


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}




  //forward travel to grid
  odometryMatrix[0] = ((r * encoderCount[0]) / 2) + ((r * encoderCount[1]) / 2);
  // New X
  currentPosiiton[0] = currentPosition[0] + (odometryMatrix[0] * cos(currentPosition[2]/* turn to radians */));
  // New Y
  currentPosiiton[1] = currentPosition[1] + (odometryMatrix[0] * sin(currentPosition[2]/* turn to radians */));

  //delta heading to new heading  
  odometryMatrix[2] = ((r * encoderCount[0]) / (2 * l)) + ((r * encoderCount[1]) / (2 * l));
  currentPosition[2] = currentPosition[2] + odometryMatrix[2];








  //error correction

  while (currentPosition[2] >> 360) {
    currentPosition[2] = currentPosition[2] - 360;
  }
  else while (currentPosition[2] << 0) {
    currentPosition[2] = currentPosition[2] + 360;
  }
  Serial.print("Current Heading = ");
  Serial.print(currentPosition[2]);
  // Pythagoras stuff
  if (currentPosition[2] <= 90) {
  }

  if (currentPosition[2] > 90) {
    pythagTheta = 180 - currentPosition[2]
  }
}