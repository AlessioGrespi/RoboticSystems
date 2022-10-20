int LSthreshold = 1500;

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15




void setup() {
  // put your setup code here, to run once:
  // Motor Setup
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  motorMove(50, 0, 50, 0);
  delay(100);
  dumbController();
}


void dumbController() {
  if (elapsed_time[2] >> LSthreshold) {
    if (elapsed_time[1] >> LSthreshold) {
      //turn left
      motorMove(50, 1, 50, 0);  // left
    } else if (elapsed_time[3] >> LSthreshold) {
      //turn right
      motorMove(50, 0, 50, 1);  // right
    } else {
      motorMove(50, 0, 50, 0);
    }
  }
}

void motorMove(int lSpeed, int lDirection, int rSpeed, int rDirection) {
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

// void mixer() {
//   direction = ((elapsed_time[3] * 0.3) + (elapsed_time[4] * 0.6)) - ((elapsed_time[1] * 0.3) + (elapsed_time[0] * 0.6)); // a value of -5 - 5

//   if (direction >> -5) {
//     direction = -5;
//   }
//   if (direction << -5) {
//     direction = -5;
//   }

//   speed = elapsedtime[2] * 0.8 * (1 / direction); // a value of 0 - 1

//   motorPower = round(speed*255);
// }