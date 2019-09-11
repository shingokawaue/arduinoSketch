//
//  MyNetSerialForMega.cpp
//  
//
//  Created by kawaue shingo on 2018/08/29.
//

#ifdef ARDUINO_AVR_MEGA2560

#include <MyNetSerialForMega.h>




    //return value : num of converted : -1 error (when the array was full
    const int MyNetSerialForMega::receiveData1(MySerialData* data) {
        uint8_t checksum = HEAD_BYTE;
        int convertedNum = 0;
        while (Serial1.available()) {//ここと
            readStock[readStockCount] = Serial1.read();//メソッドreceiveDataとの違いはここだけ
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
    }//receiveData1

#endif //ARDUINO_AVR_MEGA2560
