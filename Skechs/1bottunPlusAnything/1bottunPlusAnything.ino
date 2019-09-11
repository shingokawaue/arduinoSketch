bool before;
bool current;

int i;
int pushCount;
String s = "unpon";
void setup() {
  // put your setup code here, to run once:
  before = HIGH;
  current = HIGH;
  
pinMode(2,OUTPUT);
pinMode(4,OUTPUT);
pinMode(6,INPUT_PULLUP);

digitalWrite(2,LOW);
digitalWrite(4,LOW);
Serial.begin(9600);
}



void loop() {
  // put your main code here, to run repeatedly:

current = digitalRead(6);

if (current != before){

  if (current == LOW){//ボタン押した瞬間
    pushCount++;
    Serial.print(pushCount);
Serial.println("回押したよ");
digitalWrite(4,HIGH);
  }else{
digitalWrite(4,LOW);
  }
}
else if (current == LOW){//押してる間
  
}
before = current;
}
