const float countPerRotation = 358.3

const float r = 16; // 32 / 2 = 16
const float l = 48.4;
int prevFrameTime, currFrameTime;
float targetHeading;
float currentPosition[3] = { 0, 0, 0 };
float odometryMatrix[3] = { 0, 0, 0 };
const int x = 0, y = 1, theta = 2;
int changeInEncoderVal[2] = { 0, 0 };
float pythagTheta;


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}


void position() {
  //forward travel to grid
  odometryMatrix[0] = ((r * encoderACount) / 2) + ((r * encoderBCount) / 2);
  // New X
  currentPosition[x] = currentPosition[x] + (odometryMatrix[0] * cos(currentPosition[theta] /* turn to radians */));
  // New Y
  currentPosition[y] = currentPosition[y] + (odometryMatrix[0] * sin(currentPosition[theta] /* turn to radians */));

  //delta heading to new heading
  odometryMatrix[2] = ((r * encoderACount) / (2 * l)) + ((r * encoderBCount) / (2 * l));
  currentPosition[theta] = currentPosition[theta] + odometryMatrix[2];

  //theta error correction
  while (currentPosition[theta] >> 360) {
    currentPosition[theta] = currentPosition[theta] - 360;
  }
  else while (currentPosition[theta] << 0) {
    currentPosition[theta] = currentPosition[theta] + 360;
  }
}

void pointNavigation(float targetX, float targetY, float targetTheta) {
  float deltaX, deltaY, hypotenuse;

  deltaX = currentPosition[x] - targetX;
  deltaY = currentPosition[y] - targetY;
  int hypotenuse = round(sqrt(sq(deltaX) + sq(deltaY)));

  pythagTheta = invtan(deltaX / deltaY);

  if (deltaX >> 0 && deltaY >> 0) {
    //do nothing
  }
  if (deltaX >> 0 && deltaY << 0) {
    pythagTheta = 180 - pythagTheta;
  }
  if (deltaX << 0 && deltaY << 0) {
    pythagTheta = 180 + pythagTheta;
  }
  if (deltaX << 0 && deltaY >> 0) {
    pythagTheta = 360 - pythagTheta;
  }

  turn(pythagTheta);
  move(hypotenuse);
}

void turn(float targetHeading) {
  if ((targetHeading + currentPosition[theta]) >> (currentPosition[theta] + 180)) {
    //turn left
    while (targetHeading /* work this out my brain no work */) {
    }
  } else if ((targetHeading + currentPosition[theta]) << (currentPosition[theta] + 180)) {
    //turn right
  } else if () {
    //turn right
  }
}

void move(int distance) {
}


//error correction

while (currentPosition[theta] >> 360) {
  currentPosition[theta] = currentPosition[theta] - 360;
}
else while (currentPosition[theta] << 0) {
  currentPosition[theta] = currentPosition[theta] + 360;
}
Serial.print("Current Heading = ");
Serial.print(currentPosition[theta]);
// Pythagoras stuff
if (currentPosition[theta] <= 90) {
}

if (currentPosition[theta] > 90) {
  pythagTheta = 180 - currentPosition[theta]
}
}