// Light Sensors

#define LL_LS_PIN A11
#define L_LS_PIN A0
#define M_LS_PIN A2
#define R_LS_PIN A3
#define RR_LS_PIN A4
#define IR_LED_PWR_PIN 11

#define LS_ARRAY_LENGTH 5
int LS_ARRAY_ADDRESS[LS_ARRAY_LENGTH] = { LL_LS_PIN, L_LS_PIN, M_LS_PIN, R_LS_PIN, RR_LS_PIN };

int LSCheckSum;
bool repeatReadingBlock[LS_ARRAY_LENGTH];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    ;  //comment out when not connected to laptop
  // Light Sensor Setup
  pinMode(IR_LED_PWR_PIN, OUTPUT);
  pinMode(L_LS_PIN, INPUT);
  pinMode(M_LS_PIN, INPUT);
  pinMode(R_LS_PIN, INPUT);

  digitalWrite(IR_LED_PWR_PIN, HIGH);  // switch on IR LEDs
  Serial.println("Setup Complete");
}


void loop() {
  // put your main code here, to run repeatedly:
  irRead();
}


void irRead() {  // CONVERT TO READ AND WRITE LS ARRAY

  for (int i = 0; i < LS_ARRAY_LENGTH; i++) {
    pinMode(LS_ARRAY_ADDRESS[i], OUTPUT);
    digitalWrite(LS_ARRAY_ADDRESS[i], HIGH);
    repeatReadingBlock[i] == false;
  }

  // Tiny delay for capacitor to charge.
  delayMicroseconds(10);

  for (int l = 0; l < LS_ARRAY_LENGTH; l++) {
    pinMode(LS_ARRAY_ADDRESS[l], INPUT);
  }

  // Places to store microsecond count
  unsigned long start_time;                 // t_1
  unsigned long end_time[LS_ARRAY_LENGTH];  // t_2

  // Store current microsecond count
  start_time = micros();

  // Stay in a loop whilst the capacitor
  // is still registering as "HIGH".
  LSCheckSum = LS_ARRAY_LENGTH;
  while (LSCheckSum != 0) {
    for (int j = 0; j < LS_ARRAY_LENGTH; j++) {
      if (digitalRead(LS_ARRAY_ADDRESS[j]) == LOW && repeatReadingBlock[j] == false) {
        repeatReadingBlock[j] = true;
        end_time[j] = micros();
        LSCheckSum -= 1;
        Serial.println(LSCheckSum);
      }
    }
  }

  // Calculate elapsed time
  unsigned long elapsed_time[LS_ARRAY_LENGTH];  // t_elapsed

  for (int k = 0; k < LS_ARRAY_LENGTH; k++) {
    elapsed_time[k] = end_time[k] - start_time;
    Serial.print(elapsed_time[k]);
    if (k < LS_ARRAY_LENGTH - 1) {
      Serial.print(" | ");
    }
  }

  Serial.print("\n");

  // Delay, just so that we have time toread the
  // values when using Serial.print().
  delay(100);
}