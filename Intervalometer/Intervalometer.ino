#include <ThreadController.h>
#include <StaticThreadController.h>
#include <Thread.h>

int SER_Pin = 2;   //pin 14 on the 75HC595
int RCLK_Pin = 3;  //pin 12 on the 75HC595
int SRCLK_Pin = 4; //pin 11 on the 75HC595

const byte DIGITS[10] = {
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




// for now nreg = 0 only (should be 0/1/2)
// dig = 0 -> 9
void setDigit(int dig, int nreg) {
  byte dig_bits = DIGITS[dig];
  digitalWrite(RCLK_Pin, LOW);
  for (int nbit = 0; nbit < 8; nbit++) {
    bool dig_bit = ! bitRead(dig_bits, nbit);
    dig_bits >> 1;
  }
  digitalWrite(RCLK_Pin, HIGH);
}



int currentDigit = 2;
// updates the display eveny x ms (should be run in a pseudo-thread
// see https://github.com/ivanseidel/ArduinoThread
void updateDisplay() {
  Serial.print("updateDisplay with digit ");
  Serial.println(currentDigit);
  setDigit(currentDigit, 0);
  currentDigit = (currentDigit + 1) % 10;
  Serial.println("updateDisplay stop");
}

Thread displayThread = Thread();

void setup() {
  Serial.begin(9600);
  
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);

  displayThread.enabled = true;
  displayThread.setInterval(500);
  displayThread.onRun(updateDisplay);


  Serial.println("Initial run");
  displayThread.run();
}


void loop() {
  // Serial.println("standby");
  // nothing for now
}
