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
extern bool isFault;
extern volatile int16_t dZeroA;
extern volatile int16_t dZeroB;
extern volatile int16_t dZeroC;
#ifndef TIMER_STEP_STOP
#define TIMER_STEP_STOP    TCCR0B  = 0b00000000;
#endif

#ifndef TIMER_INIT_START
#define TIMER_INIT_START  TCCR0B  = 0b00000011;
#endif




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
  protected:
    void setCoils();
    bool initAxist(uint8_t num);
    uint8_t fallMask();
    uint8_t raiseMask();
  public:
    bool isDone();
    void init();
    uint8_t goTo(int16_t A, int16_t B, int16_t C, uint8_t coils);    
    int16_t getPosA();
    int16_t getPosB();
    int16_t getPosC();
    uint8_t getCoils();
    uint8_t getInputs();
    void proclaimPois(uint8_t num, int16_t pos);
    bool initialize();
     

    static void onRise(StepControl *control);
    static void onFall(StepControl *control);
    friend void TIMER0_COMPA_vect( void );
    friend void TIMER0_COMPB_vect( void );
};



#endif /* STEPCONTROL_H_ */