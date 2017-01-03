/*
 * SSC.cpp
 *
 * Created: 24.12.2016 18:57:08
 * Author : Sync
 */ 
 #define F_CPU 16E6
 #define _BOOTLOADER_OFFSET 0x3800
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb.h"
#include "ringbuffer.h"
#include "pincontrol.h"
#include "led/led.h"
#include "step/stepControl.h"

#define  VID 0xFF00
#define  PID 0x00FF
#define  ENABLE  PORTC |= (1<<PINC7)
#define  DISABLE PORTC &= ~(1<<PINC7)
#define TIMER_STEP_START   TCCR0B  = 0b00000010;
#define TIMER_STEP_STOP    TCCR0B  = 0b00000000;

inline void saveStatus();

#pragma  pack(1)
struct DEVICE_POINT {
  int16_t  posX;
  int16_t  posY;
  int16_t  posZ;
  uint8_t  coils;
};

struct DEVICE_STATUS {
  DEVICE_POINT point;
  uint8_t inputs;
  uint8_t status;
  uint32_t position;
};
#pragma  pack()

USB usbControl;
StepControl stepControl;
RingBufferT<DEVICE_POINT, uint8_t,3> rBuf;
volatile DEVICE_STATUS status;
volatile DEVICE_POINT dp;
volatile int16_t dZeroA = 1000;
volatile int16_t dZeroB = 1000;
volatile int16_t dZeroC = 1000;

pinIO ledA(&PORTD,PIND5);
pinIO ledB(&PORTB,PINB0);
ledControl ledCtrl;

uint32_t fullPosition = 0;
bool isInit   = false;
bool isWork   = false;
bool isError  = false;
bool isFault  = false;
bool isStartInit = false;

ISR(USB_GEN_vect, ISR_BLOCK){
  USB::onGenEvent((USB*)&usbControl);  
}

ISR(USB_COM_vect, ISR_BLOCK) { 
  USB::onComEvent((USB*)&usbControl);
}

ISR(TIMER0_COMPA_vect, ISR_NOBLOCK){
  stepControl.onFall(&stepControl);  
  TIFR0 &= ~(1<<OCR0A);
}

ISR(TIMER0_COMPB_vect, ISR_NOBLOCK){  
  stepControl.onRise(&stepControl);
  TIFR0 &= ~(1<<OCR0B);
}

ISR(TIMER1_OVF_vect,ISR_NOBLOCK){
  saveStatus();
  isWork = false;    
  TIFR1 &= ~(1<<TOV1);
}

bool onGetStatus(USB *handle){    
  handle->writeBuf((const uint8_t*)&status,sizeof(DEVICE_STATUS),16);  
  return true;  
}

bool onSetData(USB *handle){    
  uint16_t cnt = UEBCX;
  handle->readBuf((uint8_t*)&dp, cnt);

  isError = !stepControl.isDone();  
  stepControl.goTo(dp.posX, dp.posY, dp.posZ, dp.coils);
  fullPosition++;
  isWork = true;  
  return true;
}

bool onControl(USB *handle, USB_Request_Header head){
  switch(head.bRequest) {
    case 0xA0:
        TIMER_STEP_STOP;
          fullPosition = (head.wValueH<<8) | head.wValueL;
        TIMER_STEP_START;
      break;
    case 0xA1:
        TIMER_STEP_STOP;
          stepControl.proclaimPois(head.wIndexL,(head.wValueH<<8)|head.wValueL);
        TIMER_STEP_START;
      break;
    case 0xA2:
        TIMER_STEP_STOP;
        OCR0B = head.wValueL>>1;
        OCR0A = head.wValueL;        
        TIMER_STEP_START;
      break;
    case 0xA3:
        isFault = false;
        TIMER_STEP_START;
      break;
    case 0xA4:
        if( head.wIndexL == 0 ){
          dZeroA = (head.wValueH<<8)|head.wValueL;
        } else if(head.wIndexL == 1){
          dZeroB = (head.wValueH<<8)|head.wValueL;
        } else if(head.wIndexL == 2){
          dZeroC = (head.wValueH<<8)|head.wValueL;
        }
      break;
    case 0xA5:
        isStartInit = true;
      break;
  }
  return true;
}

void initUSB(){
  USB_Endpoint_Definition ep;
    
  ep.type     = INT;
  ep.dir      = IN;
  ep.size     = SIZE16;
  ep.bank     = ONE;
  ep.interval = 250;
  ep.sync     = NO_SYNC;
  ep.iType    = DATA;

  usbControl.init(VID,PID,2.0,false,true,false,500);    

  usbControl.initEndpoint(1,ep);
  ep.type     = INT;
  ep.dir      = OUT;
  ep.size     = SIZE16;
  ep.bank     = ONE;
  ep.interval = 25;
  ep.sync     = NO_SYNC;
  ep.iType    = DATA;
  usbControl.initEndpoint(2,ep);

  usbControl.registerCallback(1, (functptr) &onGetStatus);
  usbControl.registerCallback(2, (functptr) &onSetData);
  usbControl.registerControlCallback((functCtrlptr)&onControl);
}

void init(){
  DDRB    = 0xFF;
  DDRC    = 0xFF;
  DDRD    = 0b01100000;
  DDRF    = 0xFF;

  TCCR0A  = 0b00000010;
  TCCR0B  = 0; 
  TCCR1A  = 0;
  TCCR1B  = 1; 

  OCR0A   = 130;
  OCR0B   = 65;  
  TCNT0   = 0x0;  

  TIMSK0  |= (1<<OCIE0A) | (1<<OCIE0B);  
  TIMSK1  |= (1<<TOIE1);

  ledCtrl.addPin(0,&ledA);
  ledCtrl.addPin(1,&ledB);
  
}

void initStepControl(){
  stepControl.init();
  _delay_ms(50);
  ENABLE;
  TIMER_STEP_START;  
}

inline void saveStatus(){    
  status.position     = fullPosition;
  status.point.posX   = stepControl.getPosA();
  status.point.posY   = stepControl.getPosB();
  status.point.posZ   = stepControl.getPosC();
  status.point.coils  = stepControl.getCoils();
  status.inputs       = stepControl.getInputs();
  status.status       = (isFault << 3) | (isError << 2) | (isWork<<1) | (isInit);
}

int main(void) {

  init();
  initUSB();  

  while(!usbControl.isEnumerate()){
    ledCtrl.blink(0b11,150,150);
  }

  initStepControl();
  
  while (1) {
    isFault = (stepControl.getInputs() != 0);

    if( isStartInit ){
      isStartInit = false;
      isInit = stepControl.initialize();
      TIMER_STEP_STOP;
      TIMER_STEP_START;
      isFault = !isInit;  
    }

    while( isFault ){
      isFault = (stepControl.getInputs() != 0);
      ledCtrl.blink(0b01,500,500);
      ledCtrl.blink(0b11,500,500);
      if( isFault ) {
        TIMER_STEP_STOP;
      } else {
        TIMER_STEP_START;
      }
    }

    ledCtrl.blink(0b11,350,350);
  }
}

