#define CLEAR_BIT(x,n) x &= ~(1 << n)
#define MCUCR_PUDBIT_OFFSET 4 // from https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

typedef enum _GPIO_PORT_t {
  GPIO_PORT_B, // digital pin 8-13
  GPIO_PORT_C, // analog pins 
  GPIO_PORT_D  // digital pins 0-7
} GPIO_PORT_t;

typedef enum _GPIO_MODE_t {
  GPIO_MODE_INPUT = 0,
  GPIO_MODE_OUTPUT = 1,
  GPIO_MODE_INPUT_PULLUP = 2,
} GPIO_MODE_t;


GPIO_PORT_t getGpioPort(uint8_t pin) {
  if (pin >= 8 && pin <= 13) {
    return GPIO_PORT_B;
  } else if (pin >= 2 && pin <= 7) {
    return GPIO_PORT_D;
  }
  return GPIO_PORT_C;
}


void setPinMode(uint8_t pin, GPIO_MODE_t mode) {
  bool pullUp = false;
  if (mode == GPIO_MODE_INPUT_PULLUP) {
    /* need to disable the pullup disable bit in the MCUCR */
    CLEAR_BIT(MCUCR, MCUCR_PUDBIT_OFFSET);
    mode = 0; // this is for the i/o assignment with DDRx 
    pullUp = true;
  }

  switch (getGpioPort(pin)) {
    /* for each case, we need to set the correct mode using DDRx register (either input/output) and also PORTx register (for input pullup only) */
    case GPIO_PORT_B: 
    CLEAR_BIT(DDRB, pin-8);  /* port B pins start at pin 8 */
    DDRB |= (mode << pin-8); 
    PORTB |= (pullUp << pin-8);
    break;
    case GPIO_PORT_C: 
    CLEAR_BIT(DDRC, pin-14); /* port C pins start at pin 14 */
    DDRC |= (mode << pin-14); 
    PORTC |= (pullUp << pin-14);
    break;
    case GPIO_PORT_D: 
    CLEAR_BIT(DDRD, pin);  /* port D pins start at pin 0 */
    DDRD |= (mode << pin); 
    PORTD |= (pullUp << pin);
    //Serial.println("Here");
    break;
  }
}

/* Pin definitions */
const byte LED_PIN = 13;
const byte BUZZER_PIN = 6;
const byte ROTARY_SW_PIN = 2;
const byte ROTARY_CLK_PIN = 3;
const byte ROTARY_DT_PIN = 4;

#define SERIAL_BAUD_RATE 57600 // characters per second
volatile int counter = 0;
volatile bool dataChangedFlag = false;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  setPinMode(BUZZER_PIN, GPIO_MODE_OUTPUT);
  setPinMode(LED_PIN, GPIO_MODE_OUTPUT);
  setPinMode(ROTARY_SW_PIN, GPIO_MODE_INPUT_PULLUP);
  setPinMode(ROTARY_DT_PIN, GPIO_MODE_INPUT);
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW_PIN), handleButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), handleRotation, RISING);
}

void loop() {
  while (!dataChangedFlag) {};
  Serial.print("Counter: ");
  Serial.println(counter);
  dataChangedFlag = false;
}

void handleButton() {
  if (dataChangedFlag) return;
  counter = 0;
  dataChangedFlag = true;
}

void handleRotation() {
  if (dataChangedFlag) return;
  if (digitalRead(ROTARY_DT_PIN) == HIGH) {
    counter--;
  } else {
    counter++;
  }
  dataChangedFlag = true;
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