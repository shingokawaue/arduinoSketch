
String  buff;
String  PID;
String  CONECT  = "E,0,";
String  STAT;
String  CMD_;
char    KEY     = 0;
int     dust    = 0;
int     index   = 0;
int     i       = 0;
String  TD;
String  RD;
long    oneSec  = 0;

void setup() {
  Serial.begin(115200); // ハードウェアシリアルを準備
  Serial1.begin(115200);
  while (!Serial) {
  }

//  pinMode(2, OUTPUT);
  pinMode(13, OUTPUT);
//  PORTB &= ~_BV(5);
//  PORTD &= ~_BV(2);

  Serial1.println("/***  Arduino Leonardo  ***/");
  Serial1.println("/       Central            /");
  Serial1.println("/   BLE Setting now        /");
  /**************************************************************
   親機：セントラル
  **************************************************************/
  //出荷時設定
  Serial1.println("SF,1");
  //ボーレート設定
  Serial1.println("SB,1");
  //Reboot
  Serial1.println("R,1");
  //ソフトウェアシリアル変更
  Serial1.end();
  Serial1.begin(9600);
  //エコー
  Serial1.println("+");
  //Device Name
  Serial1.println("SDM,Arduino_Leonardo");
  /**************************************************************
   SS:サービス設定
   SetRegister
   0x80000000 : Device Information
  **************************************************************/
  Serial1.println("SS,80000001");
  /**************************************************************
   SR:機能設定
   SetRegister
   0x80000000 : セントラルとして開始
   0x10000000 : シリアル非同期転送サービス
   0x08000000 : MLDP外部有効
   0x02000000 : UART Flow
  **************************************************************/
  Serial1.println("SR,9A000000");
  //Reboot
  Serial1.println("R,1");
  delay(500);
  
  BLE_SCAN();

  Serial.println("/****  START BLE  ****/");
}

void loop() {
// LED_CNT();

  //相手から受信
  if (Serial1.available()){
    PORTB |= _BV(5);
    RD = Serial1.readString();
    Serial.print(RD);
    PORTB &= ~_BV(5);
  }

  //相手へ送信
  if(Serial.available()){
    PORTB |= _BV(5);
    TD = Serial.readString();
    Serial.print(TD);
    Serial1.print(TD);

    //切断コマンド
    if(!TD.compareTo("[")){
      PORTD &= ~_BV(2);
      Serial.println("Z");// Serial1.println("Z");
      Serial.println("K");// Serial1.println("K");
      Serial.println("R,1");// Serial1.println("R,1");
      BLE_SCAN();
    }
    PORTB &= ~_BV(5);
  }
}


//Bluetoothスキャン
void BLE_SCAN(){
  Rescan:

  PORTD &= ~_BV(2);
  PORTB |= _BV(5);

  //コマンド受付チェック
  Check_CMD();

  //スキャンスタート
  Serial.println("Scan..."); Serial1.println("F");

  //相手PIDチェック
  if(Check_PID()){goto Rescan;}

  //接続確認
  Serial.print(PID); Serial.print(" : Connect? y/n ");
  while(1){
    if(Serial.available()){
      KEY = Serial.read();
      Serial.println(KEY);
      if(KEY == 'y'){
        Serial.println("Now connecting...");
        break;
      }
      else if(KEY == 'n'){
        Serial.println("Now Rescanning...");
        goto Rescan;
      }else{
        Serial.println("Please 'y' or 'n' only");
      }
    }
  }

  //PID文字列変更
  CONECT.concat(PID);

  //スキャンキャンセル
  Serial.println("Scan stop"); Serial1.println("X");

  //UUID接続
  Serial.println(CONECT); Serial1.print(CONECT);

  //MLDP_PINセット
  PORTD |= _BV(2);
  Serial1.write("\n");

  //接続確認
  while(1){
    if(Serial1.available()){
      buff = Serial1.readString();
    }
    if(buff.indexOf("Connected") != -1){
      Serial.println("Connected");
      break;
    }
  }
  
  //エコー解除
  while(1){
    Serial1.println("+");
    
    if(Serial1.available()){
      buff = Serial1.readString();
    }
    if(buff.indexOf("Off") != -1){
      break;
    }
  }

  //MLDP開始
  Serial1.write("I\n");

  delay(1000);

  //不要な文字子
  if(Serial1.available()) dust = Serial1.read();
  Serial1.write(" \n");
}


//RN4020 コマンド受付チェック
void Check_CMD(){
  while(1){
    if(Serial1.available()){
      buff = Serial1.readString();
    }
    if((buff.indexOf("CMD") >= 63) || (buff.indexOf("CMD") == 0)){
      i = 0;
      break;
    }
    if(i >= 5000){
      Serial1.println("R,1");
      i = 0;
    }
    i++;
    delay(1);
  }
}


//RN420 相手PIDチェック
int Check_PID(){
  while(1){
    if(Serial1.available()){
      buff = Serial1.readString();
    }
    if((buff.indexOf("=") != -1) && (buff.indexOf(",") != -1)){
      PID = buff.substring((buff.indexOf('0')),(buff.indexOf(',',1)));
      return 0;
    }
    if(i >= 1000){
      Serial1.println("R,1");
      i = 0;
      return 1;
    }
    i++;
    delay(1);
  }
}


//RN4020 エコー確認
void Echo_Print(){
  while(1){
    if(Serial1.available()>0){
      buff = Serial1.readString();
      Serial.print(buff);
      break;
    }
  }
}


////LED
//void LED_CNT(){
//  if(millis() >= oneSec){
//    PORTB = PORTB ^ _BV(5);
//    oneSec = millis() + 1000;
////  }
//}

