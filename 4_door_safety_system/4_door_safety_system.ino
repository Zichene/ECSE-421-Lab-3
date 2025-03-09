#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

/* Pin definitions */
#define RED_LED_PIN 4
#define GREEN_LED_PIN 3
#define BLUE_LED_PIN 2
#define KEYPAD_R1 5
#define KEYPAD_R2 6
#define KEYPAD_R3 7
#define KEYPAD_R4 8
#define KEYPAD_C1 9
#define KEYPAD_C2 10
#define KEYPAD_C3 11
#define KEYPAD_C4 12
#define US_TRIG 13
#define US_ECHO 18 /* SDA pin on Uno */
#define MAG_DIG A1
#define MAG_ANA A0
#define BUZZER_S A2

/* ms Timer config parameter */
#define TIM1_OCR_VAL_DEFAULT_MS 2000 // OCR value to make TIM_ISR trigger every 1ms (for pre=8, f_clk = 16MHz)

/* Timing paramerers */
#define CTRL_WARNING_TIMEOUT 500 // tone and warning light are on for 500 ms 

/* Enum for the output of the Controller */
typedef enum _controller_status_t {
  CTRL_DISARMED,
  CTRL_ARMED,
  CTRL_ALARM,
  CTRL_NONE,
} controller_status_t;

/* Enum for the keypad FSM states */
typedef enum _keypad_state_t {
  KP_INITIAL,
  KP_1PIN,
  KP_2PIN,
  KP_3PIN,
  KP_4PIN,
  KP_CORRECT_PIN,
  KP_INCORRECT_PIN,
} keypad_state_t;

/* Enum for the keypad status */
typedef enum _keypad_status_t {
  KP_STATUS_DEFAULT = 0,
  KP_STATUS_CORRECT_PIN = 1,
  KP_STATUS_INCORRECT_PIN = 2,
} keypad_status_t;

/* Timer object, updated every MS during timer interrupt to keep track of various things */
typedef volatile struct _timer_t {
  unsigned long ms; // current value of the timer in ms
  unsigned long max_threshold_ms; // value at which the timer's started flag be reset to false
  bool started; // if timer is started or not. can be used as a flag
  bool auto_reset; // if timer auto resets when reaching max_threshold_ms
} timer_t;

/* ISR flags & global timer counters */
volatile unsigned long globalMs = 0; // keeping track of number of ms passed since beginning of program
volatile unsigned long globalUs = 0; // keeping track of number of us passed since beginning of program (will overflow after about 30 min)
volatile bool tamperDetected = false;  // set when magnetic sensor detects something

/* Global variables */
bool ultrasonic_last_echo = false;
char last_key_pressed = 0;
float last_us_measurement_cm = 0;
int presence_detected_counter = 0;
controller_status_t controller_status = CTRL_DISARMED;
controller_status_t last_controller_status = CTRL_DISARMED;
keypad_state_t keypad_state = KP_INITIAL;
keypad_status_t keypad_status = KP_STATUS_DEFAULT;
keypad_status_t last_keypad_status = KP_STATUS_DEFAULT;
int keypad_count = 0;
int keypad_N = 0;
int last_keypad_N = 0;
int num_consecutive_wrong_pins = 0;
char* correct_pin = "1234A";

/* To keep the LED blinking during CTRL_ALARM */
timer_t alarm_led_timer = {
  .ms = 0,
  .max_threshold_ms = 1000,
  .started = false,
  .auto_reset = true
};

/* To keep the buzzer buzzing during CTRL_ALARM */
timer_t alarm_timer = {
  .ms = 0,
  .max_threshold_ms = 1000,
  .started = false,
  .auto_reset = true
};

/* To keep the Controller Warning mode on for 500 ms each time a warning is activated */
timer_t ctrl_warning_timer = {
  .ms = 0,
  .max_threshold_ms = 500,
  .started = false,
  .auto_reset = true
};

/* To keep the keypad in KP_CORRECT_PIN or KP_INCORRECT_PIN for one second */
timer_t keypad_timer = {
  .ms = 0,
  .max_threshold_ms = 1000,
  .started = false,
  .auto_reset = false
};

/* When presence is detected, start timer for 15 seconds */
timer_t presence_detected_timer = {
  .ms = 0,
  .max_threshold_ms = 15000,
  .started = false,
  .auto_reset = true
};

/* When tamper is detected, start timer for 15 seconds */
timer_t tamper_detected_timer = {
  .ms = 0,
  .max_threshold_ms = 15000,
  .started = false,
  .auto_reset = true
};

/* For the ultrasonic, the ms timer is not precise enough, we need to use micros() */
unsigned long ultrasonic_timer_us = 0;
unsigned long ultrasonic_echo_timer_us = 0;

void setup() {
  /* Serial terminal */
  Serial.begin(115200);

  /* Setup pin modes */
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(KEYPAD_R1, OUTPUT);
  pinMode(KEYPAD_R2, OUTPUT);
  pinMode(KEYPAD_R3, OUTPUT);
  pinMode(KEYPAD_R4, OUTPUT);
  pinMode(KEYPAD_C1, INPUT_PULLUP);
  pinMode(KEYPAD_C2, INPUT_PULLUP);
  pinMode(KEYPAD_C3, INPUT_PULLUP);
  pinMode(KEYPAD_C4, INPUT_PULLUP);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(MAG_DIG, INPUT);
  pinMode(BUZZER_S, OUTPUT);

  /* Interrupts */
  attachPCINT(digitalPinToPCINT(MAG_DIG), handleTamper, CHANGE);

  /* Start the timer interrupts for every 1 ms */
  startGlobalTimerMS();
  
  /* Starting message */
  Serial.println("Welcome to door security system. Please enter pin: ");
}

void loop() {
  printDisplay();
  playBuzzer();
  displayLight();
  processKeyPad();
  processController();
}

/****************************************************************************
* TIMER FUNCTIONS
*****************************************************************************/

/** @brief Start the timer 
* @param timer pointer to timer_t struct
*/
void timerStart(timer_t* timer) {
  timer->started = true;
}

/** @brief Once timer is started, increment timer->ms at every tick. Once max_threshold_ms is 
*  reached, timer will auto reset if and only if the auto_reset flag is set to true.
*  @param timer pointer to timer_t struct
*/
void timerUpdate(timer_t* timer) {
  if (!timer->started) return;
  if (timer->ms > timer->max_threshold_ms && timer->auto_reset) {
    timerReset(timer);
    return;
  }
  timer->ms++;
}

/** @brief Returns true if timer->ms is strictly between t1 and t2.
* @param t1 lower bound time (ms)
* @param t2 upper bound time (ms)
* @returns bool
*/
bool timerBetween(timer_t* timer, unsigned long t1, unsigned long t2) {
  return timer->started && (t1 < timer->ms) && (timer->ms < t2);
}

/** @brief Returns true if the timer has finished.
* @param timer pointer to timer struct
* @warning should really only be used with auto_reset set to false
* @returns bool
*/
bool timerFinished(timer_t* timer) {
  return timer->ms > timer->max_threshold_ms;
}

/** @brief Resets the timer. By setting ms to 0 and the started flag to false.
* @param timer pointer to timer struct
*/
void timerReset(timer_t* timer) {
  timer->ms = 0;
  timer->started = false;
}

/* Custom implementation of micros() using TIM2 */
unsigned long getMicros() {
  return globalMs*1000 + round((1000.0f*TCNT1)/(TIM1_OCR_VAL_DEFAULT_MS));
}

/* Custom implementation of millis() using TIM2 */
unsigned long getMillis() {
  return globalMs;
}

/* initialize the timer so that it triggers an interrupt at every 1 ms  */
void startGlobalTimerMS() {
  cli(); //stop all interrupts
  // turn on CTC mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaler 8
  TCCR1B |= (1 << CS11); 
  //initialize counter value to 0;
  TCNT1  = 0;
  // set timer count: OCR = 1 ms * 16 MHz/8 = 2000
  OCR1A = TIM1_OCR_VAL_DEFAULT_MS;
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

/* stop timer */
void stopGlobalTimer() {
  cli();
  TCCR1B &= ~(1<< CS12);  // turn off the clock altogether
  TCCR1B &= ~(1<< CS11);
  TCCR1B &= ~(1<< CS10);
  sei();
  globalUs = 0; // reset globalMs
}

/* Timer ISR, update all timer objects and globalMs variable */
ISR(TIMER1_COMPA_vect) {
  timerUpdate(&alarm_led_timer);
  timerUpdate(&alarm_timer);
  timerUpdate(&ctrl_warning_timer);
  timerUpdate(&keypad_timer);
  timerUpdate(&tamper_detected_timer);
  timerUpdate(&presence_detected_timer);
  globalMs++;
}

/****************************************************************************
* GET INPUT FUNCTIONS / SENSOR ISRs
*****************************************************************************/

/* Edge capture: Will only return key if we are in a transition from 0->key*/
char getKeyPressed() {
  char key = 0;
  /* Check row 1 */
  digitalWrite(KEYPAD_R1, LOW);
  digitalWrite(KEYPAD_R2, HIGH);
  digitalWrite(KEYPAD_R3, HIGH);
  digitalWrite(KEYPAD_R4, HIGH);

  if (!digitalRead(KEYPAD_C1)) {
    key = '1';
  }
  if (!digitalRead(KEYPAD_C2)) {
    key = '2';
  }
  if (!digitalRead(KEYPAD_C3)) {
    key = '3';
  }
  if (!digitalRead(KEYPAD_C4)) {
    key =  'A';
  }

  /* Check row 2 */
  digitalWrite(KEYPAD_R1, HIGH);
  digitalWrite(KEYPAD_R2, LOW);
  if (!digitalRead(KEYPAD_C1)) {
    key =  '4';
  }
  if (!digitalRead(KEYPAD_C2)) {
    key =  '5';
  }
  if (!digitalRead(KEYPAD_C3)) {
    key =  '6';
  }
  if (!digitalRead(KEYPAD_C4)) {
    key =  'B';
  }

  /* Check row 3 */
  digitalWrite(KEYPAD_R2, HIGH);
  digitalWrite(KEYPAD_R3, LOW);
  if (!digitalRead(KEYPAD_C1)) {
    key =  '7';
  }
  if (!digitalRead(KEYPAD_C2)) {
    key =  '8';
  }
  if (!digitalRead(KEYPAD_C3)) {
    key =  '9';
  }
  if (!digitalRead(KEYPAD_C4)) {
    key =  'C';
  }

  /* Check row 4 */
  digitalWrite(KEYPAD_R3, HIGH);
  digitalWrite(KEYPAD_R4, LOW);
  if (!digitalRead(KEYPAD_C1)) {
    key =  '*';
  }
  if (!digitalRead(KEYPAD_C2)) {
    key =  '0';
  }
  if (!digitalRead(KEYPAD_C3)) {
    key =  '#';
  }
  if (!digitalRead(KEYPAD_C4)) {
    key =  'D';
  }

  if (last_key_pressed != key) {
    last_key_pressed = key;
    return key;
  }

  last_key_pressed = key; 
  return 0;
}

/* Get distance measured in cm as follows:
1: Generate 10-us duration pulse using the US_TRIG pin 
2: Speed of sound is 340 m/s  = 0.034 cm/us
3: Measure travel_time by finding the length of the pulse read in the US_TRIG pin
4: distance = speed * travel_time / 2 = 0.017 cm/us * travel_time
*/
float getUltrasonicCm() {
  if (ultrasonic_timer_us == 0) {
    ultrasonic_timer_us = micros();
  }
  if (ultrasonic_timer_us > 0 && micros() - ultrasonic_timer_us < 10) {
    digitalWrite(US_TRIG, HIGH);
  } else if (ultrasonic_timer_us > 0 && micros() - ultrasonic_timer_us > 10){
    ultrasonic_timer_us = 0;
    digitalWrite(US_TRIG, LOW);
  }

  // measure duration of pulse from ECHO pin
  bool ultrasonic_echo = digitalRead(US_ECHO);
  if (ultrasonic_echo_timer_us == 0 && ultrasonic_echo == HIGH && ultrasonic_echo != ultrasonic_last_echo) {
    ultrasonic_echo_timer_us = micros();
  }

  if (ultrasonic_echo_timer_us > 0 && ultrasonic_echo == LOW) {
    // pulse ends
    float distance_cm = 0.017 * (micros() - ultrasonic_echo_timer_us);
    ultrasonic_echo_timer_us = 0;
    ultrasonic_last_echo = ultrasonic_echo;
    return distance_cm;
  }
  ultrasonic_last_echo = ultrasonic_echo;
  return false;
}

/* Get ultrasonic measurement and check if < 100 cm */
bool isPresenceDetected() {
  float us_cm = getUltrasonicCm();
  /* Sometimes the US sensor outputs 0, so we need to check for this */
  if (us_cm <= 0) {
    return presence_detected_counter >= 50;
  }
  /* We want 50 measurements in a row to be less than 100 cm */
  if (us_cm < 100) {
    presence_detected_counter++;
  } else {
    presence_detected_counter = 0;
  }
  
  if (presence_detected_counter >= 50) {
    return true;
  } else {
    return false;
  }
}

/* ISR: only need to update if we are in CTRL_ARMED and not during a warning */
void handleTamper() {
  if (controller_status == CTRL_ARMED && !ctrl_warning_timer.started) tamperDetected = true;
}

/****************************************************************************
* MAIN LOOP FUNCTIONS
*****************************************************************************/

/* Play sound according to the controller status */
void playBuzzer() {
  if (ctrl_warning_timer.started) {
    tone(BUZZER_S, 500);
    return;
  }

  switch (controller_status) {
    case CTRL_ALARM:
    /* Start timer */
    if (!alarm_timer.started) timerStart(&alarm_timer);

    if (timerBetween(&alarm_timer, 0, 250)) tone(BUZZER_S, 1000);
    else if (timerBetween(&alarm_timer, 250, 1000)) tone(BUZZER_S, 1500);
    break;
    default:
    /* Reset tone & timer */
    noTone(BUZZER_S);
    timerReset(&alarm_timer);
    break;
  }
  last_controller_status = controller_status;
}

/* display LED color according to controller status */
void displayLight() {
  if (ctrl_warning_timer.started) {
    /* Yellow / Orange */
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, LOW);
    return;
  }

  switch (controller_status) {
    case CTRL_ARMED:
    /* Red */
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    break;

    case CTRL_ALARM:
    /* Blinking Red */
    if (!alarm_led_timer.started) timerStart(&alarm_led_timer);

    if (timerBetween(&alarm_led_timer, 0, 500)) {
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
    } else if (timerBetween(&alarm_led_timer, 500, 1000)) {
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
    }
    break;

    case CTRL_DISARMED:
    /* Green */
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, LOW);
    break;

    default:
    /* Turn off */
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    break;
  }
}

/* this function implements the FSM of the keypad component */
void processKeyPad() {
  char key = getKeyPressed();

  switch(keypad_state) {
    case KP_INITIAL:
    keypad_N = keypad_count;
    keypad_status = KP_STATUS_DEFAULT;

    if (key == correct_pin[0] && keypad_count == 0) {
      keypad_count++;
      keypad_state = KP_1PIN;
      break;
    }
    if (key && key != correct_pin[0] && keypad_count < 5 && key != '*') {
      keypad_count++;
      keypad_state = KP_INITIAL;
    }
    if (keypad_count >= 5) {
      keypad_state = KP_INCORRECT_PIN;
    }
    if (key == '*') {
      keypad_count = 0;
      keypad_state = KP_INITIAL;
    }
    break;

    case KP_1PIN:
    keypad_N = keypad_count;
    keypad_status = KP_STATUS_DEFAULT;
    if (key == correct_pin[1]) {
      keypad_count++;
      keypad_state = KP_2PIN;
      break;
    }
    if (key && key != correct_pin[1] && key != '*') {
      keypad_count++;
      keypad_state = KP_INITIAL;
    }
    if (key == '*') {
      keypad_count = 0;
      keypad_state = KP_INITIAL;
    }
    break;

    case KP_2PIN:
    keypad_N = keypad_count;
    keypad_status = KP_STATUS_DEFAULT;

    if (key == correct_pin[2]) {
      keypad_count++;
      keypad_state = KP_3PIN;
      break;
    }
    if (key && key != correct_pin[2] && key != '*') {
      keypad_count++;
      keypad_state = KP_INITIAL;
    }
    if (key == '*') {
      keypad_count = 0;
      keypad_state = KP_INITIAL;
    }
    break;

    case KP_3PIN:
    keypad_N = keypad_count;
    keypad_status = KP_STATUS_DEFAULT;

    if (key == correct_pin[3]) {
      keypad_count++;
      keypad_state = KP_4PIN;
      break;
    }
    if (key && key != correct_pin[3] && key != '*') {
      keypad_count++;
      keypad_state = KP_INITIAL;
    }
    if (key == '*') {
      keypad_count = 0;
      keypad_state = KP_INITIAL;
    }
    break;

    case KP_4PIN:
    keypad_N = keypad_count;
    keypad_status = KP_STATUS_DEFAULT;

    if (key == correct_pin[4]) {
      keypad_count++;
      keypad_state = KP_CORRECT_PIN;
      break;
    }

    if (key && key != correct_pin[4] && key != '*') {
      keypad_count++;
      keypad_state = KP_INITIAL;
    }

    if (key == '*') {
      keypad_count = 0;
      keypad_state = KP_INITIAL;
    }
    break;

    case KP_INCORRECT_PIN:
    keypad_N = 5;
    keypad_status = KP_STATUS_INCORRECT_PIN;

    /* Start timer for 1 second */
    timerStart(&keypad_timer);
    
    /* Timer reached 1 second, reset and transition to initial */
    if (timerFinished(&keypad_timer)) {
      keypad_count = 0;
      keypad_state = KP_INITIAL;
      timerReset(&keypad_timer);
    }
    break;

    case KP_CORRECT_PIN:
    keypad_N = keypad_count;
    keypad_status = KP_STATUS_CORRECT_PIN;

    /* Start timer for 1 second */
    if (!keypad_timer.started) timerStart(&keypad_timer);

    /* Timer reached 1 second, reset and transition to initial */
    if (timerFinished(&keypad_timer)) {
      keypad_count = 0;
      keypad_state = KP_INITIAL;
      timerReset(&keypad_timer);
    }
    break;

    default:
    Serial.println("ERROR: Invalid keypad state");
    break;
  }
}

/* This function updates the status of the controller based on keypad/tamper/presence signals */
void processController() {
  bool presence_detected = isPresenceDetected();

  /* If correct pin is entered, go from disarmed to armed */
  if (keypad_status == KP_STATUS_CORRECT_PIN && keypad_status != last_keypad_status && controller_status == CTRL_DISARMED) {
    controller_status = CTRL_ARMED;
    last_keypad_status = keypad_status;
  }

  /* If correct pin is entered, go from armed to disarmed */
  if (keypad_status == KP_STATUS_CORRECT_PIN && keypad_status != last_keypad_status && controller_status == CTRL_ARMED) {
    controller_status = CTRL_DISARMED;
    num_consecutive_wrong_pins = 0;
    last_keypad_status = keypad_status;
  }

  /* When controller is armed */
  if (controller_status == CTRL_ARMED) {
    if (keypad_status == KP_STATUS_INCORRECT_PIN && keypad_status != last_keypad_status) {
      Serial.println("Wrong pin");
      /* Start warning timer */
      timerStart(&ctrl_warning_timer);
      num_consecutive_wrong_pins++;
    }
    /* If the number of consecutive wrong pins goes to 3, switch to alarm state */
    if (num_consecutive_wrong_pins >= 3) {
      Serial.println("Three consecutive invalid pins!");
      controller_status = CTRL_ALARM;
      num_consecutive_wrong_pins = 0;
    }

    /* If presence is detected, start the presence timer if not already started and ctrl warning */
    if (presence_detected && !ctrl_warning_timer.started) {
      Serial.println("Presence detected!");
      /* Start warning timer */
      timerStart(&ctrl_warning_timer);
      /* Start presence timer */
      timerStart(&presence_detected_timer);
    } 

    /* If tamper is detected and ctrl warning is not active, start the tamper timer and ctrl warning */
    if (tamperDetected && !tamper_detected_timer.started && !ctrl_warning_timer.started) {
      Serial.println("Tamper detected!");
      /* Start warning timer */
      timerStart(&ctrl_warning_timer);
      /* Start tamper timer, reset flag */
      timerStart(&tamper_detected_timer);
      tamperDetected = false;
    }

    /* Two tamper events detected within 15s */
    if (tamper_detected_timer.started && tamperDetected) {
      Serial.println("Two tamper events within 15 s!");
      timerReset(&tamper_detected_timer);
      timerReset(&presence_detected_timer);
      controller_status = CTRL_ALARM;
      tamperDetected = false;
    }

    /* Tamper and presence detected within 15 seconds */
    if (tamper_detected_timer.started && presence_detected_timer.started) {
      Serial.println("Tamper and presence within 15 seconds!");
      timerReset(&tamper_detected_timer);
      timerReset(&presence_detected_timer);
      controller_status = CTRL_ALARM;
    }
  }

  /* When the controller is in alarm state */
  if (controller_status == CTRL_ALARM) {
    if (keypad_status == KP_STATUS_CORRECT_PIN && keypad_status != last_keypad_status) {
      controller_status = CTRL_DISARMED;
    }
  }
  last_keypad_status = keypad_status;
}

/* Prints the display to serial monitor */
void printDisplay() {
  if (keypad_N > 0) {
    if (last_keypad_N != keypad_N) {
      Serial.print('*');
      last_keypad_N = keypad_N;
    }
  } else if (keypad_N == 0 && keypad_N != last_keypad_N) {
    Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\nEnter Pin: ");
    last_keypad_N = keypad_N;
  }
}


