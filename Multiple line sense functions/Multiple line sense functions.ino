// Light Sensors

#define LL_LS_PIN A11
#define L_LS_PIN A0
#define M_LS_PIN A2
#define R_LS_PIN A3
#define RR_LS_PIN A4
#define IR_LED_PWR_PIN 11

#define LS_ARRAY_LENGTH 5
int LS_ARRAY_ADDRESS[LS_ARRAY_LENGTH] = { LL_LS_PIN, L_LS_PIN, M_LS_PIN, R_LS_PIN, RR_LS_PIN };

int LSCheckSum, test = 0;
bool repeatReadingBlock[LS_ARRAY_LENGTH];

void irRead(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    ;  //comment out when not connected to laptop
  // Light Sensor Setup
  pinMode(IR_LED_PWR_PIN, OUTPUT);
  pinMode(LL_LS_PIN, INPUT);
  pinMode(L_LS_PIN, INPUT);
  pinMode(M_LS_PIN, INPUT);
  pinMode(R_LS_PIN, INPUT);
  pinMode(RR_LS_PIN, INPUT);

  digitalWrite(IR_LED_PWR_PIN, HIGH);  // switch on IR LEDs
  Serial.println("Setup Complete");
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("yoooo");
  irRead();
  Serial.println("reeeee");
 test++;
 Serial.println(test);
 if (test >> 3)
 {
   while(1);
 }
}


void irRead() {  // CONVERT TO READ AND WRITE LS ARRAY
  Serial.println("READ FUNC CALL");
  for (int i = 0; i < LS_ARRAY_LENGTH; i++) {
    pinMode(LS_ARRAY_ADDRESS[i], OUTPUT);
    digitalWrite(LS_ARRAY_ADDRESS[i], HIGH);
    repeatReadingBlock[i] == false;
    Serial.print("Charging ");
    Serial.println(LS_ARRAY_ADDRESS[i]);
    Serial.println(i);
  }

  // Tiny delay for capacitor to charge.
  delayMicroseconds(10);

  for (int j = 0; j < LS_ARRAY_LENGTH; j++) {
    pinMode(LS_ARRAY_ADDRESS[j], INPUT);
    Serial.print("INPUT ");
    Serial.println(LS_ARRAY_ADDRESS[j]);
    Serial.println(j);
  }

  // Places to store microsecond count
  unsigned long start_time;                 // t_1
  unsigned long end_time[LS_ARRAY_LENGTH];  // t_2

  // Store current microsecond count
  start_time = micros();

  // Stay in a loop whilst the capacitor
  // is still registering as "HIGH".
  LSCheckSum = LS_ARRAY_LENGTH;
  Serial.print("LSCheckSum ");
  Serial.println(LSCheckSum);
  while (LSCheckSum != 0) {
    for (int k = 0; k < LS_ARRAY_LENGTH; k++) {
      if (digitalRead(LS_ARRAY_ADDRESS[k]) == LOW && repeatReadingBlock[k] == false) {
        repeatReadingBlock[k] = true;
        end_time[k] = micros();
        LSCheckSum -= 1;
        Serial.print("LSCheckSum ");
        Serial.println(LSCheckSum);
      }
      Serial.println(k);
    }
  }
  Serial.print("LSCheckSum ");
  Serial.println(LSCheckSum);
  // Calculate elapsed time
  unsigned long elapsed_time[LS_ARRAY_LENGTH];  // t_elapsed

  for (int l = 0; l < LS_ARRAY_LENGTH; l++) {
    elapsed_time[l] = end_time[l] - start_time;
    Serial.print(elapsed_time[l]);
    //Serial.print("ARRAY LENGTH ");
    //Serial.println(LS_ARRAY_LENGTH);
    if (l < LS_ARRAY_LENGTH - 1) {
      Serial.print(" | ");
    }
    //Serial.println(l);
  }

  Serial.println(" aaaaaaaa");

  delay(1000);

  // Delay, just so that we have time toread the
  // values when using Serial.print().
}