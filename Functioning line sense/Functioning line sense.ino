// Light Sensors

#define LS_LEFT_PIN A0
#define M_LS_PIN A2
#define R_LS_PIN A3
#define IR_LED_PWR_PIN 11
#define LS_ARRAY_ADDRESS [3] = { L_LS_PIN, M_LS_PIN, R_LS_PIN }

void setup() {
  // put your setup code here, to run once:

  // Light Sensor Setup
  pinMode(IR_LED_PWR_PIN, OUTPUT);
  pinMode(LS_LEFT_PIN, INPUT);
  pinMode(M_LS_PIN, INPUT);
  pinMode(R_LS_PIN, INPUT);
  digitalWrite(IR_LED_PWR_PIN, HIGH);  // switch on IR LEDs
  Serial.begin(9600);
  while (!Serial); //comment out when not connected to laptop
  Serial.print("Setup Complete \n");
  //delay(5000);
}


void loop() {
  // put your main code here, to run repeatedly:
  irRead();
}


void irRead() {  // CONVERT TO READ AND WRITE LS ARRAY

  // Charge capacitor by setting input pin
  // temporarily to output and HIGH
  pinMode(LS_LEFT_PIN, OUTPUT);
  digitalWrite(LS_LEFT_PIN, HIGH);

  // Tiny delay for capacitor to charge.
  delayMicroseconds(10);

  //  Turn input pin back to an input
  pinMode(LS_LEFT_PIN, INPUT);

  // Places to store microsecond count
  unsigned long start_time;  // t_1
  unsigned long end_time;    // t_2

  // Store current microsecond count
  start_time = micros();

  // Stay in a loop whilst the capacitor
  // is still registering as "HIGH".
  while (digitalRead(LS_LEFT_PIN) == HIGH) {
    // Do nothing, waiting for LS_LEFT_PINT to
    // go to LOW state.
  }

  // LS_LEFT_PIN must have gone LOW,
  // store microsecond count afterwards
  end_time = micros();

  // Calculate elapsed time
  unsigned long elapsed_time;  // t_elapsed
  elapsed_time = end_time - start_time;

  // Print output.
  Serial.print("Left line sensor: ");
  Serial.print(elapsed_time);
  Serial.print("\n");

  // Delay, just so that we have time toread the
  // values when using Serial.print().
  delay(10);
}