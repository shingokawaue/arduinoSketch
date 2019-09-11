
const int swichLED_Pin = 4;
bool beforeConnection = false;
String str = "";
//初期化
void setup() {

pinMode(13,OUTPUT);
pinMode(swichLED_Pin,OUTPUT);
Serial.begin(9600); //USBのやつ　9600bpsで通信開始
Serial1.begin(115200); //DIGITAL0番,1番ピンをｼﾘｱﾙ通信で使用,RN4020(ｼﾝｸﾞﾙﾓｰﾄﾞBluetooth smartﾓｼﾞｭｰﾙ)と通信開始
// シリアルストリームが開いていないときには、何もしない:
while (!Serial) ;
Serial.println("始まるよ!!");
digitalWrite(13,LOW);
}

void loop() {

    bool sw1, sw2;//センサー状況？
    int volt;
    
  //コマンド”Ｅ”を送る。接続出来るまで。
  if(!isRN4020Connected()){

//    establish_A_ConnectionRN4020();//(これを呼び出さなくてもなぜか接続出来る・・。  
     digitalWrite(13,LOW);  
        delay(5000); 
        beforeConnection = false;      
    return;
  }
  
  //接続している
  if (beforeConnection == false){
  Serial.println("接続してます");
    beforeConnection == true;
    digitalWrite(13,HIGH);
  }
  
  
  
  delay(2000);
  //test
    

  sendCommand("LC");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);


      
  sendCommand("LS");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);



   sendCommand("SS,06600000");
  delay(100);
  str = readAllSerial1();
        Serial.println(str);
//        
//       sendCommand("SN,RN4020-GOMIKAS-");
//      str = readAllSerial1();
//          Serial.println(str);

//      
//        sendCommand("S-");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str);
//
//delay(100);
//      
//        sendCommand("SB");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str);
//delay(100);
//      
//        sendCommand("SDM");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str);
//delay(100);
//      
//        sendCommand("SDN");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str);
//    
//        sendCommand("SDS");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str);
//    
//        sendCommand("SM");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str);
//    
//        sendCommand("SR");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str); 
//      
//      sendCommand("ST");//ClientHandleRead
//  str = readAllSerial1();
//      Serial.println(str);
      
  sendCommand("CHR,000B");//サーバ側（Iphone）の　1810（血圧）サービスの　
  //2A35というｷｬﾗｸﾀﾘｽﾃｨｯｸのﾊﾝﾄﾞﾙ（アドレス）
  delay(100);
  str = readAllSerial1();
      Serial.println(str);

      
        sendCommand("CHR,001A");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);

      
   sendCommand("CURC,000B");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);
      
         sendCommand("CURC,2A35");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);    
       
    sendCommand("CURC,001A");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);
      
      sendCommand("CURC,2A2B");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);     
 
      
  sendCommand("CURV,2A19");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);

  sendCommand("SUR");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);

  sendCommand("SHR");
  delay(100);
  str = readAllSerial1();
      Serial.println(str);       
                      
      //3０秒毎
    delay(15 * 1000);  
  return;    //toriaezu
  
    
    //センサー類読み取り
    sw1 = true;
    sw2 = true;
    
    //PIO2/PIO3の読み取り
//    str = readRN4020("000D",1);//ｷｬﾗﾘﾀﾘｽﾃｨｯｸ値の読み出し
//    
//    if(sw_val & 0x02){//&演算子は、ビット単位で双方が共に真の場合実行。
//      //ASCIIコード。0x02はSTX（テキスト開始）
//      sw1 = false; 
//    }
//    if(sw_val & 0x04){//&演算子は、ビット単位で双方が共に真の場合実行。
//      //ASCIIコード。0x04はEOT（転送終了）
//      sw2 = false; 
//    }
//    
//    //AIO0の読み取り
//    volt = readRN4020("000B",2);
//    // この辺でWiFiClientSecure使ってWebにつなげてデータをインターネットに
//    // UPする処理を入れる
  

}



void establish_A_ConnectionRN4020(){
  Serial1.println("E");//RN4020にコマンド"E"送信（接続を確立）
  delay(100);
  while(Serial1.available()){//ちょい待ち、なんか来たら読み込む
    Serial1.read();  
  }
}






///////////////////////////////////////////////////////////////////////////////////////
//メソッドの定義
///////////////////////////////////////////////////////////////////////////////////////

bool isRN4020Connected(){


  bool ret = false;
  String line;

  Serial1.println("D");//RN4020へ"設定内容をダンプ出力"コマンド
  // とりあえず解析に必要な情報を受信するまで待つ
  for(int i = 0; i < 50; i++)
  {
    delay(10);

    Serial.print(i);

    
    if(Serial1.available() <= 54){    //Serial.available():シリアルポートに何バイトのデータが到着しているかを返します
      continue;
    }else{
      break;
    }
  }
  if(Serial1.available() <= 54){
    Serial.println("待ってもデータ来ないわ・・・・");
    goto EXIT_FUNC;
  }

  
  //54バイト以上バッファがたまったら解析開始

  //Ｄコマンドでダンプさせた設定内容は以下の形式になるので
  //BTA=???
  //Name=???
  //Connected=no???
  //Connected=???
  line = readlnSerial1(); 
  Serial.println(line);
  if(!line.startsWith("BTA=")){ goto EXIT_FUNC; }
  line = readlnSerial1(); 
  Serial.println(line);
  if(!line.startsWith("Name=")){ goto EXIT_FUNC; }
  line = readlnSerial1();
  Serial.println(line);

  //iPhoneに接続してなくてもここまではくる
  if(line.startsWith("Connected=no")){ goto EXIT_FUNC; }
  if(!line.startsWith("Connected=")){ goto EXIT_FUNC; }
ret = true;
// 解析完了残りを読み捨て
EXIT_FUNC:
  while(Serial1.available()){//while文は評価が偽ならばループを抜ける
    Serial1.read();  
  }
  return ret;
}



///////////////////////////////////////////////////////////////////////////////////////

void sendCommand (String str){
Serial1.println(str);
Serial.println("send Command [" + str + "] and wait 10ms");
  delay(10);
}



///////////////////////////////////////////////////////////////////////////////////////


//String send_CommandRN4020(String id){
//  int val = -1;
//  String str;
//  int retry_cnt = 0;
//
//  //ゴミを捨てる
////RETRY:
//  while(Serial1.available()){
//    Serial1.read();  
//  }
//  // コマンド発行
//  Serial1.println("CHR," + id);//コマンド、"CHR,id" クライアント ハンドルから値を読み出し
// //ハンドルでアドレス指定してリモートデバイスからクライアント サービスの
////キャラクタリスティックの内容を読み出します。
////パラメータ(ここではid）は、クライアント サービスのキャラクタリスティック
////に対応するハンドルの16ビット16進数値です。（000Ｄ，000Ｂ）
////このコマンドは、ピアデバイスとのアクティブなリンクが存在し、ハンドル パラメー
////タが有効で、対応するキャラクタリスティックのプロパティが読み出し可能な場合の
////み有効です。戻り値は、リモートのピアデバイスから取得します。
//Serial.print("send [CHR,");
//Serial.print(id);
//Serial.print("] and wait 100ms ⇒ ");
//  delay(100);
//  if(Serial1.available() <= 0){
//    return "RN4020_return_nothing...";
//  }
////  c = Serial1.read();
////  if(c != 'R') {
////    if(retry_cnt >= 5){
////      goto EXIT_FUNC;
////    }
////    retry_cnt++;
////    goto RETRY;
////  }
////
////  c = Serial1.read();
////  if(c != ',') { goto EXIT_FUNC;}
////  val = 0;
////  for(int i = 0; i < (2 * num); i++) {
////    c = Serial1.read();
////    int tmp = 0;
////    if((c >= 'A') && (c <= 'F')){
////      tmp = c - 'A' + 10;  
////    }else if((c >= '0') && (c <= '9')){
////      tmp = c - '0';  
////    }
////    val = ( val << 4) + tmp;
////  }
//EXIT_FUNC:
//  while(Serial1.available()){
//    str = str + Serial1.read();  
//  }
//  return str;
//}

///////////////////////////////////////////////////////////////////////////////////////

String readlnSerial1(){
  String line = String("");
  char c;

  while(Serial1.available()){//改行0x0aで抜ける
    c = Serial1.read();
    if( c == 0x0A ){//改行で抜ける
      break;
    }
    line = line + String(c);
  }

  return line;
}

///////////////////////////////////////////////////////////////////////////////////////

String readAllSerial1(){
  String str = String("");
  char c;


for(int i = 0;i < 1000;i++){
  
  while(Serial1.available()){  
    c = Serial1.read();
    str = str + String(c);
  }

}



  return str;
}


