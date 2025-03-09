const byte LED_PIN = 13;
const byte BUZZER_PIN = 6;
const byte ROTARY_SW_PIN = 5;
const byte ROTARY_CLK_PIN = 3;
const byte ROTARY_DT_PIN = 4;
volatile bool ledOn = false;
volatile bool dataChangedFlag = false;
volatile uint16_t ocrval = 0;

#define LED_FREQUENCY_DEFAULT 5000
#define LED_OCR_VAL_DEFAULT 100

/********************************************
DRIVER CODE FOR SETMODE
*********************************************/
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

void handleButton() {
  if (dataChangedFlag) return;
  ocrval = LED_OCR_VAL_DEFAULT;
  dataChangedFlag = true;
}

void handleRotation() {
  if (dataChangedFlag) return;
  if (digitalRead(ROTARY_DT_PIN) == HIGH) {
    if (ocrval-10 > 0) {
      ocrval -= 10;
    }
  } else {
    ocrval += 10;
  }
  dataChangedFlag = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  setPinMode(BUZZER_PIN, GPIO_MODE_OUTPUT);
  setPinMode(LED_PIN, GPIO_MODE_OUTPUT);
  setPinMode(ROTARY_SW_PIN, GPIO_MODE_INPUT_PULLUP);
  setPinMode(ROTARY_DT_PIN, GPIO_MODE_INPUT);
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW_PIN), handleButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), handleRotation, RISING);
  initTimer1();
  Serial.println(LED_OCR_VAL_DEFAULT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!dataChangedFlag) {};
  //Serial.print("OCRval: ");
  //Serial.println(ocrval);
  Serial.print("Duty Cycle (%): ");
  Serial.println(((float)LED_OCR_VAL_DEFAULT/((float)(LED_OCR_VAL_DEFAULT+ocrval))));
  dataChangedFlag = false;
}

void changeOCR1(uint16_t OCR1val) {
  // turn off interrupts
  //cli();
  /* change value of OCR1A */
  OCR1A = OCR1val;
  // sei();
}

/* initialize the timer (timer 1) at default frequency */
void initTimer1() {
  cli(); //stop all interrupts
  // turn on CTC mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);

  // Set CS12 bit for prescaler 256
  TCCR1B |= (1 << CS12); 
  
  //initialize counter value to 0;
  TCNT1  = 0;
  
  // set timer count for given default frequency
  OCR1A = LED_OCR_VAL_DEFAULT; // = (16*10^6) / (5000*256) = 12

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}

/* Timer ISR */
ISR(TIMER1_COMPA_vect) {
 //write your timer code here
 if (ledOn) {
  digitalWrite(LED_PIN, HIGH);
  changeOCR1(LED_OCR_VAL_DEFAULT);
 } else {
  digitalWrite(LED_PIN, LOW);
  changeOCR1(ocrval);
 }
 ledOn = !ledOn;
}




