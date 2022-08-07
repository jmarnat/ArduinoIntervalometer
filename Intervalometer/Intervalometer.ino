#define DISPLAY_DELAY_MS 5           // for time-multiplexing the 3 digits
#define DEBOUNCE_DELAY_MS 10         // push-button debounce time

// These are all for controlling a LTD056B 3 digits 7 segments display
const int PIN_SER = 2;               // pin 14 on the 75HC595 (shift-register)
const int PIN_RCLK = 3;              // pin 12 on the 75HC595
const int PIN_SRCLK = 4;             // pin 11 on the 75HC595
const int PIN_DIGITS[] = {5, 6, 7};  // 3 anodes

const int PIN_BUTTON = 8;            // click button

const int PIN_RELAY = 12;            // output relay

const byte DIGITS[10] = {            // 7 segments bits (DP=0)
  0b11111100,
  0b01100000,
  0b11011010,
  0b11110010,
  0b01100110,
  0b10110110,
  0b10111110,
  0b11100000,
  0b11111110,
  0b11110110
};

byte digitsBuffer[3] = {5, 0, 0};   // 3 digits buffer (they need to be kept in memory)
byte decimalPoint = 0;              // TODO

int shutter_open_millis  = 5000;    // open shutter duration
int shutter_close_millis = 1000;    // closed shutter duration between cycles
int shutter_state = 0;              // see bellow

#define STATE_STOP  0
#define STATE_OPEN  1
#define STATE_PAUSE 2

// the actual intervalometer timers
unsigned long timer_start;
unsigned long timer_current;
int IntervalometerState = STATE_STOP;

// for debounding the push button
unsigned long debounce_start;
unsigned long debounce_current;

// writing a single digit (time-multiplexed)
void setDigit(int dig, int nreg) {
  for (int reg = 0; reg < 3; reg++) {
    if (reg == nreg) digitalWrite(PIN_DIGITS[reg], HIGH);
    else digitalWrite(PIN_DIGITS[reg], LOW);
  }

  byte dig_bits = DIGITS[dig];
  digitalWrite(PIN_RCLK, LOW);
  for (int nbit = 0; nbit < 8; nbit++) {
    bool dig_bit = ! bitRead(dig_bits, nbit);
    digitalWrite(PIN_SRCLK, LOW);
    digitalWrite(PIN_SER, dig_bit);
    digitalWrite(PIN_SRCLK, HIGH);
    dig_bits >> 1;
  }
  digitalWrite(PIN_RCLK, HIGH);
}

// for time-multiplexing the display of the 3 digits
int currentDigit = 0;
void updateDisplay() {
  setDigit(digitsBuffer[currentDigit], currentDigit);
  currentDigit = (currentDigit + 1) % 3;
}

boolean buttonPushedPrev = LOW;
boolean buttonPushedCurrent = LOW;

unsigned long lastDisplayTime = millis();
unsigned long currentDisplayTime = millis();

// open the shutter for image capture
void openShutter() {
  digitalWrite(PIN_RELAY, LOW);
}

// closing the shutter
void closeShutter() {
  digitalWrite(PIN_RELAY, HIGH);
}

// start the intervalometer
void StartIntervalometer() {
  Serial.println("START INTERVALOMETER");
  IntervalometerState = STATE_OPEN;
  openShutter();
  timer_start = millis();
}

// pause (of close the shutter)
void PauseIntervalometer() {
  Serial.println("PAUSE INTERVALOMETER");
  IntervalometerState = STATE_PAUSE;
  closeShutter();
  timer_start = millis();
}

// true stop
void StopIntervalometer() {
  Serial.println("STOP INTERVALOMETER");
  IntervalometerState = STATE_STOP;
  digitalWrite(PIN_RELAY, HIGH);
}

// setting up the Arduino
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 3; i++) {
    pinMode(PIN_DIGITS[i], OUTPUT);
  }
  
  pinMode(PIN_SER, OUTPUT);
  pinMode(PIN_RCLK, OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);

  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);



  
  Serial.println("Initial run");
  digitalWrite(PIN_RELAY, HIGH);

  StopIntervalometer();
  debounce_start = millis();
}

// main loop
void loop() {
  // updating display
  currentDisplayTime = millis();
  if ((currentDisplayTime - lastDisplayTime) > DISPLAY_DELAY_MS) {
    updateDisplay();
    lastDisplayTime = currentDisplayTime;
  }

  // checking buttons
  buttonPushedCurrent = digitalRead(PIN_BUTTON);
  debounce_current = millis();
  if ((buttonPushedCurrent != buttonPushedPrev) && ((debounce_current - debounce_start) > DEBOUNCE_DELAY_MS)) {
    if (buttonPushedCurrent == HIGH) {
      if (IntervalometerState == STATE_STOP) {
        StartIntervalometer();
      } else {
        StopIntervalometer();
      }
    }
    buttonPushedPrev = buttonPushedCurrent;
    debounce_start = debounce_current;
  }

  // intervalometer loop
  if (IntervalometerState == STATE_OPEN) {
    timer_current = millis();
    if ((timer_current - timer_start) > shutter_open_millis) {
      PauseIntervalometer();
    }
  } else if (IntervalometerState == STATE_PAUSE) {
    timer_current = millis();
    if ((timer_current - timer_start) > shutter_close_millis) {
      StartIntervalometer();
    }
  }
}
