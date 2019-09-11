int latchPin = 8;  // 74HC595のST_CPへ
int clockPin = 12; // 74HC595のSH_CPへ
int dataPin = 11;  // 74HC595のDSへ
 
void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}
 
void loop() {
  // LED1からLED8までを順に光らせます
  for (int j = 0; j < 8; j++) {
    // 送信中のlatchPinはグランド(LOW)レベル
    digitalWrite(latchPin, LOW);
    // シフト演算を使って点灯するLEDを選択しています
    shiftOut(dataPin, clockPin, LSBFIRST, 1<<j);//LSBFIRST:LOWBYTEから読み込む
    // 送信終了後latchPinをHIGHにする
    digitalWrite(latchPin, HIGH);
    delay(100);
  }
}
