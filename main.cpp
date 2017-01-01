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
#define TIMER_POS_START   TCCR1B  = 0b00000010;
#define TIMER_POS_STOP    TCCR1B  = 0b00000000;
#define TIMER_STEP_START  TCCR0B  = 0b00000010;
#define TIMER_STEP_STOP   TCCR0B  = 0;

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
DEVICE_POINT dp;

pinIO ledA(&PORTD,PIND5);
pinIO ledB(&PORTB,PINB0);


pinIO dirA(&PORTF,PINF4);
pinIO dirB(&PORTF,PINF1);
pinIO dirC(&PORTF,PINF0);

ledControl ledCtrl;

uint32_t fullPosition = 0;
bool isInit = false;
bool isWork = false;
bool isError = false;

ISR(USB_GEN_vect, ISR_BLOCK){
  USB::onGenEvent((USB*)&usbControl);  
}

ISR(USB_COM_vect, ISR_BLOCK) { 
  USB::onComEvent((USB*)&usbControl);
}

ISR(TIMER0_COMPA_vect, ISR_NOBLOCK){
  uint8_t mask = stepControl.onFall(&stepControl) & 0b111;
  PORTF &= (0b00011111) | ( mask << 5);  
  TIFR0 &= ~(1<<OCR0A);
}

ISR(TIMER0_COMPB_vect, ISR_NOBLOCK){
  PORTF |= 0b11100000;
  stepControl.onRise(&stepControl);
  TIFR0 &= ~(1<<OCR0B);
}

bool onGetStatus(USB *handle){    
  handle->writeBuf((const uint8_t*)&status,sizeof(DEVICE_STATUS),16);  
  return true;  
}

bool onSetData(USB *handle){    
  uint16_t cnt = UEBCX;
  uint8_t mask = PORTF & 0b11100000;

  handle->readBuf((uint8_t*)&dp, cnt);
  isError = !stepControl.isDone();  
  mask = stepControl.goTo(dp.posX, dp.posY, dp.posZ, dp.coils);
  if( mask & (1<<2) ) {
    mask |= (1<<4);
  }
  PORTF = mask;

  fullPosition++;
  isWork = true;
  
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
}

void init(){
  DDRB    = 0xFF;
  DDRC    = 0xFF;
  DDRD    = 0b11100000;
  DDRF    = 0xFF;

  TCCR0A  = 0b00000010;
  TCCR0B  = 0;  

  OCR0A   = 130;
  OCR0B   = 65;  
  TCNT0   = 0x0;  

  TIMSK0  |= (1<<OCIE0A) | (1<<OCIE0B);  

  ledCtrl.addPin(0,&ledA);
  ledCtrl.addPin(1,&ledB);

  //PORTB |= 0x60 | 0x80 | 0x40 | 0x10;
}

void initStepControl(){
  stepControl.init(&dirA,&dirB,&dirC);
  _delay_ms(50);
  ENABLE;
  TIMER_STEP_START;  
}

inline void saveStatus(){  
  
  status.position   = fullPosition;
  status.point.posX = stepControl.getPosA();
  status.point.posY = stepControl.getPosB();
  status.point.posZ = stepControl.getPosC();
  status.point.coils = stepControl.getCoils();
  status.inputs      = 0xAA;
  status.status      = (isError << 2) | (isWork<<1) | (isInit);
}

int main(void) {

  init();
  initUSB();  

  while(!usbControl.isEnumerate()){
    ledCtrl.blink(0b11,150,150);
  }

  initStepControl();
  
  
  while (1) {
    saveStatus();    
    isWork = false;
    ledCtrl.blink(0b01,1500,500);    
  }
}

