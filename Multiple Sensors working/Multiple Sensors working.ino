#define LL_LS_PIN A11
#define L_LS_PIN A0
#define M_LS_PIN A2
#define R_LS_PIN A3
#define RR_LS_PIN A4
#define IR_LED_PWR_PIN 11

void setup() {
  pinMode(IR_LED_PWR_PIN, OUTPUT);

  pinMode(LL_LS_PIN, INPUT);
  pinMode(L_LS_PIN, INPUT);
  pinMode(M_LS_PIN, INPUT);
  pinMode(R_LS_PIN, INPUT);
  pinMode(RR_LS_PIN, INPUT);

  digitalWrite(IR_LED_PWR_PIN, HIGH);  // switch on IR LEDs

  Serial.begin(9600);
  while (!Serial)
    ;  //comment out when not connected to laptop
  Serial.print("Setup Complete \n");
}

void loop() {
  irRead();
}

void irRead() {
  int LS_ARRAY_ADDRESS[5] = { LL_LS_PIN, L_LS_PIN, M_LS_PIN, R_LS_PIN, RR_LS_PIN };

  for (int j = 0; j < 5; j++) {
    pinMode(LS_ARRAY_ADDRESS[j], OUTPUT);
    digitalWrite(LS_ARRAY_ADDRESS[j], HIGH);
  }

  delayMicroseconds(10);

  pinMode(LL_LS_PIN, INPUT);
  pinMode(L_LS_PIN, INPUT);
  pinMode(M_LS_PIN, INPUT);
  pinMode(R_LS_PIN, INPUT);
  pinMode(RR_LS_PIN, INPUT);

  unsigned long start_time;
  unsigned long end_time[5];

  start_time = micros();

  while (digitalRead(LL_LS_PIN) == HIGH)
    //write check all sensor function with end time inside
    ;

  while (digitalRead(L_LS_PIN) == HIGH)
    //write check all sensor function with end time inside
    ;

  while (digitalRead(M_LS_PIN) == HIGH)
    //write check all sensor function with end time inside
    ;

  while (digitalRead(R_LS_PIN) == HIGH)
    //write check all sensor function with end time inside
    ;

  while (digitalRead(RR_LS_PIN) == HIGH)
    //write check all sensor function with end time inside
    ;

  unsigned long elapsed_time[5];

  for (int i = 0; i < 5; i++) {
    end_time[i] = micros();
    elapsed_time[i] = end_time[i] - start_time;
    Serial.print(elapsed_time[i]);
    Serial.print(" | ");
  }

  Serial.println("");
}