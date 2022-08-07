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
  Serial.print("digit: ");
  Serial.println(dig);
  
  byte dig_bits = DIGITS[dig];
  Serial.println(dig_bits);

  digitalWrite(RCLK_Pin, LOW);
  for (int nbit = 0; nbit < 8; nbit++) {
    bool dig_bit = ! bitRead(dig_bits, nbit);
    Serial.print(dig_bit);
    digitalWrite(SRCLK_Pin, LOW);
    digitalWrite(SER_Pin, dig_bit);
    digitalWrite(SRCLK_Pin, HIGH);
    dig_bits >> 1;
  }
  digitalWrite(RCLK_Pin, HIGH);
  Serial.println();
  Serial.println();
}



void setup() {
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);

  Serial.begin(9600);

  setDigit(2, 0);
}


void loop() {
  for (int d = 0; d <= 9; d++) {
    setDigit(d, 0);
    delay(500);
  }
  delay(500);
}
