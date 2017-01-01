/*
 * led.h
 *
 * Created: 28.12.2016 23:10:42
 *  Author: Sync
 */ 


#ifndef LED_H_
#define LED_H_

#include "../pincontrol.h"

class ledControl{
  protected:
    pinIO *ledPin[8];    
  public:
    void addPin(uint8_t num, pinIO *control);
    void blink(uint8_t leds,uint16_t tOn, uint16_t tOff);

};




#endif /* LED_H_ */