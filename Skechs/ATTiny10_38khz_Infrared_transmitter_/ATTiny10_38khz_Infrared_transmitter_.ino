#include <avr/io.h>
#define F_CPU 8000000UL//_delay_ms()を使用するため（ここでCPUクロックを設定できるわけではなく、_delay_ms_()の参照用
#include <util/delay.h>//_delay_ms()を使用するため

void setup() {
  //クロックの設定
  CCP = 0xD8;//Configuration Change Protection Register 
  //IOREG CLKMSR CLKPSR WDTCSR などのレジスタを書き換えるためにはCCPに0xD８を書き込み、４CPUクロック以内に書き換えなければならない
  CLKMSR = 0x00;//Clock Main Settings Register 00:Calibrated Internal 8MHzOscillatorを選択(デフォルト)
  CCP = 0xD8;//Configuration Change Protection Register 
  CLKPSR = 0x00;//Clock Prescaler Register 分周数を１に ※デフォルトは８ クロックは8mhzになる
  
//waveformの設定
 TCCR0A = 0b10000010;//Timer/Counter0 Control Register A     :OC0AコンペアマッチでLOW出力 OC0Bはdisconnected
TCCR0B = 0b00011001;//Timer/Counter0 Control Register B    :InputCaptureNoiseCancelerオフ　高速PWM動作 TOP値はICR0　分周なし
ICR0 = 210;//TOP値(タイマー0の１周期)の指定　38kHzに出来るだけ合わせる
//タイマーの１周期　= 0.000000125s(CPUクロック8MHzの１周期) * 211 = 26.375μs  38kHz = 26.315.78μs
OCR0A = 30;//コンペアマッチAに使われる値（高速PWM コンペアマッチAでLOW出力の設定なので、
//OCR0Aの値がタイマーのカウントと一致したタイミングでOC0AがHIGHからLOWに変わる）つまり、OCR0Aの値でデューティ比が決まる
// 0~69 HIGH 70 ~210 LOWとなり、デューティ比は約1/3となる
DDRB = 0b00001111; //PORTB(I/Oポート) 全て出力に（Arduinoの pinMode(0~3,OUTPUT);と同じ)
}

void loop() {//30msごとに赤外線LED38kHzをオンオフする
_delay_ms(1);
TCCR0A = 0b00000010;//コンペアマッチオフ(赤外線LEDを発光させない)
_delay_ms(29);
TCCR0A = 0b10000010;//コンペアマッチオン(赤外線LEDを発光させる)
}
