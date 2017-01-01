/*
 * stepControl.h
 *
 * Created: 29.12.2016 22:43:52
 *  Author: Sync
 */ 


#ifndef STEPCONTROL_H_
#define STEPCONTROL_H_

#include <avr/interrupt.h>
#include "../pincontrol.h"
extern "C" void TIMER0_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER0_COMPB_vect(void) __attribute__ ((signal));

class StepControl{
  private:    
    int16_t posA, posB, posC;
    int16_t newA, newB, newC;
    uint16_t cntA, cntB, cntC;
    uint16_t setA, setB, setC;    
    uint8_t mask;
    uint8_t nowCoils;
    int8_t  dirA, dirB, dirC;
    
    bool    isOnPoint;
    
    pinIO   *dirStepA, *dirStepB, *dirStepC;
  protected:
    void setCoils();
  public:
    bool isDone();
    void init(pinIO *DIR_A, pinIO *DIR_B, pinIO *DIR_C);
    uint8_t goTo(int16_t A, int16_t B, int16_t C, uint8_t coils);    
    int16_t getPosA();
    int16_t getPosB();
    int16_t getPosC();
    uint8_t getCoils();

    static uint8_t onRise(StepControl *control);
    static uint8_t onFall(StepControl *control);
    friend void TIMER0_COMPA_vect( void );
    friend void TIMER0_COMPB_vect( void );
};



#endif /* STEPCONTROL_H_ */