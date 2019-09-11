// SRF02のアドレスを変更

#include <Wire.h>

byte original_address = 60;  // 変更前のアドレス: 0xE0
byte new_address = 61;       // 変更後のアドレス: 0xE2

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Change address begin ...");

  byte original_i2c_address = original_address >> 1;
  Wire.beginTransmission(original_i2c_address);
  Wire.write(0x00);
  Wire.write(0xA0);
  Wire.endTransmission();

  Wire.beginTransmission(original_i2c_address);
  Wire.write(0x00);
  Wire.write(0xAA);
  Wire.endTransmission();

  Wire.beginTransmission(original_i2c_address);
  Wire.write(0x00);
  Wire.write(0xA5);
  Wire.endTransmission();

  Wire.beginTransmission(original_i2c_address);
  Wire.write(0x00);
  Wire.write(new_address);
  Wire.endTransmission();
  Serial.println("Change address finish!");
}

void loop() {
}
