//Aruduino用 シリアル通信用　パケット作成クラス

//パケット構造
//0~7bit :ヘッダバイト0x7E 01111110
//9~13 : MCID 5bit 0~63(MicroComputerId 送信者識別No
//14~18: MCID 5bit 0~63 (MicroComputerId 受信者識別No
//19~22: SDPP  4bit (SendPurpose  送信目的コード :センサーの数値報告　異常検出　定期確認信号 命令　文章
//23~29: PID 7bit 0~127 (PartsId パーツ識別No
//30~35: DTCT 6bit 0~63 (DataContents) データ内容 :距離　温度　オンオフ
//36~39: DTFM 4bit 0~31 (DataForm)データ形式 : float byte配列 char char* など
//40~ データ
//:チェックサムフッタ

//この後aruduinoでパリティビット、ストップビットを加えるのでデータ形式は変わるはず
#ifndef _MYNETSERIAL_H_
#define _MYNETSERIAL_H_




#include <Arduino.h>


//ヘッダファイルでのdefineは競合の可能性がある
#define HEAD_BYTE (uint8_t)0x7E //01111110
#define FOOT_BYTE 0x8E
#define ESCAPE_BYTE 0x7D//01111101
#define ESCAPE_MASK 0x20//00100000 ビットの排他論理和 ^ を２回すると同じ値に戻る!!
//01011101 1回目　(ESCAPE_BYTEとESCAPE_MASKの排他論理和)
//01111101 2回目

//MCID 5bit 0~63(MicroComputerId
#define MCID_NOT_SPECIFIED 0x00//00000000
#define MCID_MASTER_MEGA 0x01//00000001
#define MCID_GATE_UNO 0x02//00000002
//SDPP  4bit 0~31 (SendPurpose  送信目的コード
#define SDPP_SENSOR_VALUE_REPORT 0x01//値報告
#define SDPP_ABNORMALITY＿DETECTION 0x02//異常検出
#define SDPP_PERIODIC_CONFIRMATION_COMMUNICATION 0x03//定期確認連絡
//PID 7bit 0~127 (PartsId パーツ識別No
#define PID_GATE_DTC1 0x01//DTC = DistanceSensor
#define PID_GATE_DTC2 0x02
#define PID_GATE_DTC3 0x03
#define PID_GATE_DTC4 0x04
#define PID_GATE_DTC5 0x05
#define PID_GATE_UNO_CPU 0x06
#define PID_GATE_UNO_VCC 0x07
//DTFM 4bit 0~31 (DataForm)データ形式
#define DTFM_FLOAT 0x01
//DTCT 6bit 0~127
#define DTCT_NO 0x00
#define DTCT_DISTANCE_CM 0x01
//isContaind
#define IS_CONTAINED_NO 0x00
#define IS_CONTAINED_YES 0x01
#define IS_CONTAINED_ERROR 0x02


struct MySerialData {
  uint8_t isContained = 0x00;
  uint8_t senderMCID = 0x00;
  uint8_t receiverMCID = 0x00;
  uint8_t sdpp = 0x00;
  uint8_t pid = 0x00;
  uint8_t dtct = 0x00;
  uint8_t dtfm = 0x00;
  uint8_t value[4];
  MySerialData() {
    memset(value, 0x00, sizeof(value));
  }
};

class MyNetSerial {
  public:
    MyNetSerial(uint8_t mcid);//コンストラクタ　使用者のMCIDを設定
    void sendData(uint8_t receiver_mcid , uint8_t sdpp, uint8_t pid , uint8_t dtct , float val);
    const int receiveData(MySerialData* data);
  protected:
    const bool convertData(MySerialData* data);
    bool checksumCheck();
    uint8_t myid;
    uint8_t* readStock = new uint8_t[100];
    uint8_t readStockCount = 0;
};//class

inline MyNetSerial::MyNetSerial(uint8_t mcid)
{
  myid = mcid;
  memset(readStock, 0x00, sizeof(readStock));
}

#endif //_MYNETSERIAL_H_
