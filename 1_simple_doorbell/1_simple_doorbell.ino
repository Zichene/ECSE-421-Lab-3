/* Pin definitions */
const byte LED_PIN = 5;
const byte BUZZER_PIN = 6;
const byte ROTARY_SW_PIN = 2;

#define SERIAL_BAUD_RATE 57600 // characters per second

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ROTARY_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW_PIN), buzz, FALLING);
}

void loop() {
  delay(1);
}

/**
* @brief ISR. This function is entered any time the value read at the ROTARY_SW_PIN changes from HIGH to LOW.
*/
void buzz() {

  /* turn LED on to show that we are in the ISR */
  digitalWrite(LED_PIN, HIGH);
  
  /* Higher frequency buzz followed by lower frequency buzz using delayPrintln() */
  analogWrite(BUZZER_PIN, 100);
  delayPrintln(500);
  analogWrite(BUZZER_PIN, 25);
  delayPrintln(500);

  /* turn off LED and buzzer to show that we have exited the ISR */
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

/**
* @brief Approximation of delay() function that leverages the Serial terminal to generate blocking delay.
* @param ms Number of milliseconds of the delay.
*/
void delayPrintln(unsigned long ms) {
  /* It takes about 50 div BAUD_RATE seconds to send out one character */
  unsigned long numCharacters = round(SERIAL_BAUD_RATE * ms/(50000)); 
  for (int i = 0; i < numCharacters; i++) {
    Serial.println(i);
  }
}



