#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);// set the LCD address to: 
//(0x3F Smraza) or (0x27 HiLetGo, Sainsmart & Sunfounder) for a 20 chars and 4 line display

void setup()
{
lcd.begin(16,2);// initialize the lcd
lcd.backlight();

   delay(100);
   lcd.noBacklight();
   delay(100);
   lcd.backlight();
lcd.setCursor(0,0);// first number is position, second is row
lcd.print("Welcome 2 Ur LCD :)");
lcd.setCursor(0,1);
lcd.print("Arduino & LCM IIC 2004");
}

void loop()
{
}
