#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

/* pin setup for LCD */
LiquidCrystal lcd(2,3,4,5,6,7);

/* other pins */
#define JOYSTICK_SW_PIN 9
#define JOYSTICK_VRX_PIN A0
#define JOYSTICK_VRY_PIN A1

/* Timer defines to generate 1 ms */
#define TIM1_OCR_VAL_DEFAULT 2000 //  1m * 16M / 8
#define TIM0_OCR_VAL_DEFAULT 250  //  1m * 16M / 64

/* joystick thresholds */
#define LEFT_THRESHOLD  800 // more than this will be registered as left
#define RIGHT_THRESHOLD 400 // less than this will be registered as right
#define UP_THRESHOLD    400 // less than this will be registered as up
#define DOWN_THRESHOLD  800 // more than this will be registered as down

/* game parameters */
#define GAME_MAX_TIME_MS  10000 // 10 seconds of game time 

/* Enum used for the joystick direction */
typedef enum _direction_t {
  LEFT = 0,
  RIGHT = 1,
  UP = 2,
  DOWN = 3,
  BUTTON = 4,
  NONE = 5,
} direction_t;

/* ISR flags & timer counters */
volatile unsigned long globalMs = 0; // keeping track of number of ms passed since last: for timer 1
volatile unsigned long globalMs_t0 = 0; // keeping track of number of ms passed since last: for timer 0 
volatile bool timerStarted = false; // keeping track of if the timer is started or not: for timer 1
volatile bool timerStarted_t0 = false; // keeping track of if the timer is started or not: for timer 0
volatile bool gameStarted = false; // keeping track of if the game has started
volatile bool gameRunning = false; // keeping track of if the game is currently running
volatile bool joystickButtonPressed = false; // if the button is pressed

/* Other global variables */
direction_t lastDirectionRead = NONE; // last direction read by the get_direction() function
unsigned long trialCount = 0; // count the number of trials 
unsigned long errorCount = 0; // count the number of errors during the game
double totalReactionTime = 0; // sum up the reaction times


void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);	
  Serial.begin(57600);
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);

  /* Only one interrupt used here, for the joystick SW */
  attachPCINT(digitalPinToPCINT(JOYSTICK_SW_PIN), handleButton, FALLING);

  /* Reset timers */
  stopTimer();
  stopTimer_0();
}


void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press button to");
  lcd.setCursor(0, 1);
  lcd.print("  start game!");

  myDelay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Last run: ");
  lcd.setCursor(0, 1);
  lcd.print("E:");
  lcd.print(errorCount);
  lcd.print(" T:");
  lcd.print(trialCount);
  lcd.print(" A:");
  if (trialCount != 0) lcd.print(totalReactionTime/(1000*trialCount));
  else lcd.print(0);

  myDelay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Game duration:");
  lcd.setCursor(0, 1);
  lcd.print(GAME_MAX_TIME_MS/1000);
  lcd.print(" sec");

  myDelay(1000);

  while (gameRunning) {
    /* Reset stats */
    trialCount = 0;
    errorCount = 0;
    totalReactionTime = 0;

    /* Countdown to get ready */
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Get ready ... ");

    myDelay(1000);
    lcd.clear();
    lcd.print("             3");
    
    myDelay(1000);
    lcd.clear();
    lcd.print("             2");

    myDelay(1000);
    lcd.clear();
    lcd.print("             1");

    myDelay(1000);

    startTimer(); // start global timer, game automatically ends when we reach MAX_GAME_TIME

    while (globalMs < GAME_MAX_TIME_MS) { 
      // get random direction 
      direction_t dir = random(0, 5);

      startTimer_0();
      joystickButtonPressed = false;
      lcd.setCursor(0, 1);
      while (true) {
        if (globalMs_t0 % 250 == 0) lcd.clear(); 
        lcdPrintDirection(dir);
        lcd.setCursor(0, 1);
        lcd.print("Time ms: ");
        lcd.print(globalMs_t0);
        
        /* Wait for correct user input and increase error count if needed */
        direction_t userEntered = getJoystickDirection();
        if (userEntered != NONE) {
          if (userEntered == dir) {
            break;
          } else {
            errorCount++;
          }
        }

        /* Handle button press (flag from ISR) */
        if (joystickButtonPressed && dir == BUTTON) {
          joystickButtonPressed = false;
          break;
        }

        /* Increase error count when prompt is not button but button is pressed */
        if (joystickButtonPressed && dir != BUTTON) {
          errorCount++;
          joystickButtonPressed = false;
        }

        /* Handle main timer overflow while waiting for user input */
        if (globalMs > GAME_MAX_TIME_MS) {
          gameRunning = false;
          break;
        }
      }

      if (!gameRunning) {
        stopTimer_0();
        break; // go to game over
      }

      trialCount++;
      lcd.clear(); // clear screen to indicate that successful 
      totalReactionTime += stopTimer_0();
      joystickButtonPressed = false;
    }
    stopTimer();
    lcd.clear();
    lcd.print("Game over.");
    myDelay(3000);

    lcd.setCursor(0, 0);
    lcd.print("Number of errors: ");
    lcd.setCursor(0, 1);
    lcd.print(errorCount);
    myDelay(5000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Number of trials: ");
    lcd.setCursor(0, 1);
    lcd.print(trialCount);
    myDelay(5000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Avg time per trial (s): ");
    lcd.setCursor(0, 1);
    lcd.print(totalReactionTime/(1000*trialCount));
    myDelay(5000);

    gameRunning = false;
  }
}

/* Used in polling mode, returns the direction of the joystick if any is detected (based on thresholds) */
/* Only update on edge: NONE -> direction */
direction_t getJoystickDirection() {
  int x_value = analogRead(JOYSTICK_VRX_PIN);
  int y_value = analogRead(JOYSTICK_VRY_PIN);
  direction_t direction = NONE;

  if (y_value > LEFT_THRESHOLD) {
    direction = LEFT;
  }

  if (y_value < RIGHT_THRESHOLD) {
    direction = RIGHT;
  }

  if (x_value > DOWN_THRESHOLD) {
    direction = DOWN;
  }

  if (x_value < UP_THRESHOLD) {
    direction = UP;
  }

  if (lastDirectionRead != direction) {
    lastDirectionRead = direction;
    return direction;
  }

  lastDirectionRead = direction;
  return NONE;
}

void lcdPrintDirection(direction_t dir) {
  switch (dir) {
        case LEFT:
        lcd.print("       <=");
        break;
        case RIGHT:
        lcd.print("       =>");
        break;
        case UP:
        lcd.print("       ^");
        break;
        case DOWN:
        lcd.print("       v");
        break;
        case BUTTON:
        lcd.print("       +");
        break;
  }
}

/* Since the native delay() function uses T0, we have to reimplement it */
void myDelay(unsigned long ms) {
  startTimer_0();
  while(globalMs_t0 < ms) {};
  stopTimer_0();
}

void handleButton() {
  if (!gameRunning) {
    gameRunning = true;
  }
  joystickButtonPressed = true;
}


void changeOCR1(uint16_t OCR1val) {
  /* change value of OCR1A */
  OCR1A = OCR1val;
}

/* initialize the timer so that it triggers an interrupt at every 1 ms  */
void startTimer() {
  cli(); //stop all interrupts
  // turn on CTC mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);

  // Set CS12 bit for prescaler 8
  TCCR1B |= (1 << CS11); 
  
  //initialize counter value to 0;
  TCNT1  = 0;
  
  // set timer count: OCR = 1 ms * 16 MHz/8 = 2000
  OCR1A = TIM1_OCR_VAL_DEFAULT;

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  timerStarted = true;
}

void startTimer_0() {
  cli();
  // Turn on CTC mode
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A |= (1 << WGM01); //  ATmega328P, table 14-8.

  // Set CS01 and CS00 bits for prescaler 64
  TCCR0B &= ~(1 << CS02);
  TCCR0B |= (1 << CS01);
  TCCR0B |= (1 << CS00); 

  // set counter to 0
  TCNT0 = 0;

  // set OCR to correct value to generate 1ms interrupts
  OCR0A = TIM0_OCR_VAL_DEFAULT;

  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();
  timerStarted_t0 = true;
}

double stopTimer_0() {
  double retval = globalMs_t0 + ((double)TCNT0)/TIM0_OCR_VAL_DEFAULT;
  cli();

  /* turn off the clock */
  TCCR0B &= ~(1 << CS02);
  TCCR0B &= ~(1 << CS01);
  TCCR0B &= ~(1 << CS00); 

  /* reset counter to 0 */
  TCNT0 = 0;

  sei();
  globalMs_t0 = 0;
  timerStarted_t0 = false;
  return retval;
}

/* stop timer and return time passed as a double, reset the globalMs value */
double stopTimer() {
  double retval = globalMs + ((double)TCNT1)/TIM1_OCR_VAL_DEFAULT;
  cli();

  /* turn off the clock */
  TCCR1B &= ~(1<< CS12);
  TCCR1B &= ~(1<< CS11);
  TCCR1B &= ~(1<< CS10);

  /* reset counter to 0 */
  TCNT1 = 0;

  sei();
  globalMs = 0; // reset globalMs
  timerStarted = false;
  return retval;
}

/* Timer ISRs */

/* Global counter for game time */
ISR(TIMER1_COMPA_vect) {
  globalMs++;
}

/* Counter used to time each symbol */
ISR(TIMER0_COMPA_vect) {
  globalMs_t0++;
}




