#include "MyNetSerial.h"



void MyNetSerial::sendData(uint8_t receiver_mcid , uint8_t sdpp, uint8_t pid , uint8_t dtct , float val) {
  uint8_t dataBytes[9];

  //パケット構造
  //0~7bit :ヘッダバイト0x7E 01111110
  //以下dataBytes
  //9~13 : MCID 5bit 0~63(MicroComputerId 送信者識別No
  //14~18: MCID 5bit 0~63 (MicroComputerId 受信者識別No
  //19~22: SDPP  4bit (SendPurpose  送信目的コード :センサーの数値報告　異常検出　定期確認信号 命令　文章
  //23~29: PID 7bit 0~127 (PartsId パーツ識別No
  //30~35: DTCT 6bit 0~63 (DataContents) データ内容 :距離　温度　オンオフ
  //36~39: DTFM 4bit 0~31 (DataForm)データ形式 : float byte配列 char char* など
  //40~ データ
  //以上dataBytes
  //:チェックサム
  //フッタ

  //DataBytesの作成
  dataBytes[0] =
  {
    (myid << 3)//送信者識別MCID 5bitを左詰
    + (receiver_mcid >> 2)//受信者識別５bitの上３桁を右詰
  };
  dataBytes[1] =
  {
    (receiver_mcid << 5)//受信者識別５bitの下2桁を左詰
    + (sdpp << 1)//送信目的SDPP 4bit を2~6桁目に
    + (pid >> 6)//パーツ識別No PID 7bitの上２桁を右詰
  };
  dataBytes[2] =
  {
    (pid << 2)//パーツ識別No PID 7bitの下５桁を左詰
    + (dtct >> 3 )//データ内容　DTCT 6bitの上３桁を右詰
  };
  float databuf[1];
  databuf[0] = val;
  dataBytes[3] =
  {
    (dtct << 5 )//データ内容　DTCT 6bitの下３桁を左詰
    + (DTFM_FLOAT << 1 )//データ形式　4bitを4~7桁目に
    + ( ((uint8_t*)databuf)[0] >> 7 )//以下 floatデータ詰め
  };
  dataBytes[4] =
  {
    ( ((uint8_t*)databuf)[0] << 1 )
    + ( ((uint8_t*)databuf)[1] >> 7 )
  };
  dataBytes[5] =
  {
    ( ((uint8_t*)databuf)[1] << 1 )
    + ( ((uint8_t*)databuf)[2] >> 7 )
  };
  dataBytes[6] =
  {
    ( ((uint8_t*)databuf)[2] << 1 )
    + ( ((uint8_t*)databuf)[3] >> 7 )
  };
  dataBytes[7] =
  {
    ( ((uint8_t*)databuf)[3] << 1 )
    + ( ((uint8_t*)databuf)[4] >> 7 )
  };
  dataBytes[8] =
  {
    ( ((uint8_t*)databuf)[4] << 1 )
  };

  //ヘッダ、チェックサム、フッタをつけて送信
  uint8_t checksum = 0;
  Serial.write(HEAD_BYTE);
  checksum += HEAD_BYTE;
  for (uint8_t i = 0; i < 9; ++i)
  {
    if ((dataBytes[i] == ESCAPE_BYTE) || (dataBytes[i] == HEAD_BYTE) || (dataBytes[i] == FOOT_BYTE))
    {
      Serial.write(ESCAPE_BYTE);
      checksum += ESCAPE_BYTE;
      Serial.write(dataBytes[i] ^ ESCAPE_MASK);
      checksum += dataBytes[i] ^ ESCAPE_MASK;
    }
    else
    {
      Serial.write(dataBytes[i]);
      checksum += dataBytes[i];
    }
  }
  Serial.write(checksum); //Checksum
  Serial.write(FOOT_BYTE);//フッタ
}//sendData


//return value : num of converted : -1 error (when the array was full
const int MyNetSerial::receiveData(MySerialData* data) {
  uint8_t checksum = HEAD_BYTE;
    int convertedNum = 0;
  while (Serial.available()) {
    readStock[readStockCount] = Serial.read();
    readStockCount++;

    if (readStock[readStockCount - 1] == FOOT_BYTE) { //フットバイトが来たら検証変換する
      checksum += FOOT_BYTE;
      if (checksum == readStock[readStockCount - 2]) {
          if (convertData(data) == -1){
              return -1;//when the array was full
          }
      } else {
        //data.isContained = IS_CONTAINED_ERROR;//通信検証エラーを追加
      }
        convertedNum++;
      continue;
    }
    if (readStock[readStockCount - 1] == HEAD_BYTE) //ヘッドバイトが来たらreadStockを０埋め
    {
      memset(readStock[0], 0x00, sizeof(readStock));
      readStockCount = 0;
      continue;
    }
    if (readStock[readStockCount - 1] == ESCAPE_BYTE)
    {
      checksum += ESCAPE_BYTE;
      uint8_t nextByte = Serial.read();
      readStock[readStockCount - 1] = nextByte ^ ESCAPE_MASK;
      checksum += nextByte;
    }
    else {
      checksum += readStock[readStockCount - 1];
    }
  }//while
    return convertedNum;
}//receiveData

//パケット構造
//0~7bit :ヘッダバイト0x7E 01111110
//以下dataBytes
//9~13 : MCID 5bit 0~63(MicroComputerId 送信者識別No
//14~18: MCID 5bit 0~63 (MicroComputerId 受信者識別No
//19~22: SDPP  4bit (SendPurpose  送信目的コード :センサーの数値報告　異常検出　定期確認信号 命令　文章
//23~29: PID 7bit 0~127 (PartsId パーツ識別No
//30~35: DTCT 6bit 0~63 (DataContents) データ内容 :距離　温度　オンオフ
//36~39: DTFM 4bit 0~31 (DataForm)データ形式 : float byte配列 char char* など
//40~ データ
//以上dataBytes
//:チェックサム
//フッタ

//return value :true converted : false array was full
const bool MyNetSerial::convertData(MySerialData* data) {//readStockのデータ部分をMySerialData構造体に変換する
    int enternum;
    if(data[0].isContained == IS_CONTAINED_NO){
        enternum = 0;
    }else if(data[1].isContained == IS_CONTAINED_NO){
        enternum = 1;
    }
    else if(data[2].isContained == IS_CONTAINED_NO){
        enternum = 2;
    }
    else if(data[3].isContained == IS_CONTAINED_NO){
        enternum = 3;
    }
    else{
        return false;//when the array was full
    }
    
    uint8_t buf;
  //4バイトデータの場合のreadStockのパケット状態
  //この時点でヘッダは抜けている
  //[0]SSSSSRRR S:senderID R:receiverID D:sendPurpose P:partsID C:dataContents F:dataForm 0~:データ
  //[1]RRDDDDPP
  //[2]PPPPPCCC
  //[3]CCCFFFF0
  //[4]00000001
  //[5]11111112
  //[6]22222223
  //[7]3333333
  //[8]cheksum
  //[9]フッタ
  //                       SSSSSRRR >> 3 = 000SSSSS
  data[enternum].senderMCID = (readStock[0] >> 3);
  //                       SSSSSRRR   &  ~0b11111000 = 00000RRR
  //                                             00000RRR << 2 = 000RRR00
  //                                                                    RRDDDDPP >> 6 = 000000RR
  data[enternum].receiverMCID = ((readStock[1] &  ~0b11111000)  << 2 ) | (readStock[1] >>6);
                        data[enternum].sdpp = ( (readStock[1] & ~0b11000000) >> 2);
                       data[enternum].pid = ((readStock[1] & ~0b11111100) << 5) | (readStock[2] >> 3);
                       data[enternum].dtct = ((readStock[2] & ~0b11111000) << 3) | (readStock[3] >> 5);
                                     data[enternum].dtfm = ((readStock[3] & 0b11100000) >> 1);
  for (int i = 0; i < (readStockCount - 5) ; i++) {
  data[enternum].value[i] = (readStock[i + 3] << 7) | (readStock[i + 4] >> 1);
  }
  data[enternum].isContained = IS_CONTAINED_YES;
    return true;
}//convertData
