//
//  Header.h
//  
//
//  Created by kawaue shingo on 2018/08/29.
//

#ifndef _MYNETSERIAL_FOR_MEGA_H_
#define _MYNETSERIAL_FOR_MEGA_H_
#ifdef ARDUINO_AVR_MEGA2560//MEGA2560の時だけ以下のコード実行

#include <MyNetSerial.h>
class MyNetSerialForMega : public MyNetSerial{
public:
    MyNetSerialForMega(uint8_t mcid);//aruduino環境ではデフォルトコンストラクタが作られない？
    const int receiveData1(MySerialData* data);
private:
    
};
inline MyNetSerialForMega::MyNetSerialForMega(uint8_t mcid):MyNetSerial(mcid){}

#endif // ARDUINO_AVR_MEGA2560
#endif /* _MYNETSERIAL_FOR_MEGA_H_ */


