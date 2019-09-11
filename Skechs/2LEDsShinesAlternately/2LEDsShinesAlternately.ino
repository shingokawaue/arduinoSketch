
int green = 13; 
int yellow = 12; 
int blue = 11;
void setup() {
  // put your setup code here, to run once:
pinMode(green , OUTPUT);
pinMode(yellow , OUTPUT);
pinMode(blue , OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(green,HIGH);
digitalWrite(blue,LOW);
delay(5000);
digitalWrite(green,LOW);
digitalWrite(yellow,HIGH);
delay(5000);
digitalWrite(yellow,LOW);
digitalWrite(blue,HIGH);
delay(5000);

}
