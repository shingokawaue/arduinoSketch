//PB0が入力、PB1が出力 PB2がLED

#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIM0_CAPT_vect){//タイマーがICR0の値に達した時 100msの間赤外線を受信してない
  PORTB |= 0b00000100; //PB2をHIGH
  PORTB &= 0b11111101; //PB1をLOW
}
ISR(PCINT0_vect){//PinChangeInterruput 赤外線を受信している時に起こる
  PORTB |= 0b00000010; //PB1をHIGH
  PORTB &= 0b11111011; //PB2をLOW
  TCNT0 = 0;//
}
void setup() {
  CCP = 0xD8;//Configuration Change Protection Register 
  //IOREG CLKMSR CLKPSR WDTCSR などのレジスタを書き換えるためにCCPに0xD８を書き込み４CPUクロック以内に書き換え
  CLKMSR = 0x00;//Clock Main Settings Register 00:CalibratedInternal 8 MHzOscillator
  CCP = 0xD8;//Configuration Change Protection Register 
  CLKPSR = 0b00000011;//Clock Prescaler Register 分周数を8に(デフォルト) クロックは1Mhzになる
 TCCR0A = 0b00000000;//Timer/Counter0 Control Register A
 //OC0A,OC0B 共にdisconnected
TCCR0B = 0b00011001;
//InputCaptureNoiseCancelerオフ　WGM：1100 ICR0がTOPのCTCモード Clock Select:001分周なし(1Mhz)
ICR0 = 99999;//TOP値の指定 これにより　タイマー０の１周期　= 1μs * 100000 = 100msとなる  (赤外線送信機の方は38khzを20msごとにトグルする

//Interrupt
TIMSK0 = 0b00100000;//Timer/Counter0 Interrupt Mask Register   (Timer/Counter0のInputCaptureInterruptを有効に 
//タイマーがICR0の値に到達した時に割り込み発生

PCICR = 0b00000001;//Pin Change Interrupt Control Register  （PinChangeInterruptを有効にする
PCMSK = 0b00000001;//Pin Change Mask Register  (PCINT0の割り込みを有効に

DDRB = 0b0001110; //PORTB PB0を入力　PB1~3出力
PORTB = 0b00000001;//PB0をプルアップ

sei();//割り込み許可
}

void loop() {
}
