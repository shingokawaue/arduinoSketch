#ifndef LedControl_MyExtention_h
#define LedControl_MyExtention_h

#include <LedControl.h>
#include <matlixChars.h>
#include <string.h>
#define DIR_0 0
#define DIR_90 1
#define DIR_180 2
#define DIR_270 3

class LedControl_MyExtention : public LedControl {
    
 public:
LedControl_MyExtention(int dataPin, int clkPin, int csPin, int numDevices=1):LedControl(dataPin,clkPin,csPin,numDevices){};
    /* 
     * Display a character on a 8x8Matlix display.
     * Params:
     * addr	address of the display
     * value	the character to be displayed. 
     * direction  the direction to be displayed.
     */
    void setCharMat(int addr,char value,int direction = DIR_180);
    /* 
     * Display a character on a 8x8Matlix display.
     * Params:
     * addr	address of the lead display
     * value	the character string to be displayed. 
     * direction  the direction to be displayed.
     */
    void setStringMat(int addr,String value ,int direction = DIR_180);
    void setIntMat(int addr, int value,int direction = DIR_180);
    
    void setInt(int addr, int digit, int value);//7segment display
};

#endif //LedControl_MyExtention_h
