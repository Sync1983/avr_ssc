/*
 * led.cpp
 *
 * Created: 28.12.2016 23:10:31
 *  Author: Sync
 */ 

 #include <util/delay.h>
 #include "led.h"
 #include "../pincontrol.h"

 void ledControl::addPin(uint8_t num, pinIO *control){
    if( num > 7 ){
      return;
    }
    this->ledPin[num] = control;
 }

 void ledControl::blink(uint8_t leds,uint16_t tOn, uint16_t tOff){
    uint8_t mask,i;
    for(i = 0; i<7; i++){
      mask = 1<<i;
      if( ( (leds&mask) == mask ) && (this->ledPin[i] != 0) ){
        this->ledPin[i]->Set();
      }
    }

    while(tOn--){
      _delay_us(1000);
    }

    for(i = 0; i<7; i++){
      mask = 1<<i;
      if( ( (leds&mask) == mask ) && (this->ledPin[i] != 0) ){
        this->ledPin[i]->Clr();
      }
    }

    while(tOff--){
      _delay_us(1000);
    }
 }