/*
 * stepControl.cpp
 *
 * Created: 29.12.2016 22:44:01
 *  Author: Sync
 */ 
#include <avr/interrupt.h>
#include <stdlib.h>
#include "stepControl.h"

void StepControl::init(){  
  posA = posB = posC = 0;
  setA = setB = setC = newA = newB = newC = 0;  
}

bool StepControl::isDone(){
  return this->isOnPoint;
}

void StepControl::setCoils(){
  uint8_t mask = PORTB & (0b00010000);
  mask |= (this->nowCoils&(1<<3))?0x80:0;
  mask |= (this->nowCoils&(1<<2))?0x60:0;
  mask |= (this->nowCoils&(1<<1))?0x40:0;
  mask |= (this->nowCoils&(1<<0))?0x10:0;
  PORTB = mask;
}

int16_t StepControl::getPosA(){
  return (this->posA >> 1);
}

int16_t StepControl::getPosB(){
  return (this->posB >> 1);
}

int16_t StepControl::getPosC(){
  return (this->posC >> 1);
}

uint8_t StepControl::getCoils(){
  return this->nowCoils;
}

void StepControl::proclaimPois(uint8_t num, int16_t pos){
  if( num == 0 ){
    this->posA = this->newA = pos*2;
  } else if( num == 1 ){
    this->posB = this->newB = pos*2;
  } else if( num == 2){
    this->posC = this-> newC= pos*2;
  }
}

uint8_t StepControl::goTo(int16_t A, int16_t B, int16_t C, uint8_t coils){    
  const float fMax = 200.0f;
  float a,b,c;
  uint8_t mask = PORTF & 0b11101100;

  newA = A*2;
  newB = B*2;
  newC = C*2;

  a = abs(posA - newA);
  b = abs(posB - newB);
  c = abs(posC - newC);
  if( (a > fMax)||(a == 0) ) a = fMax;
  if( (b > fMax)||(b == 0) ) b = fMax;
  if( (c > fMax)||(c == 0) ) c = fMax;

    
  cntA = setA = 5 + (fMax / a);
  cntB = setB = 5 + (fMax / b);
  cntC = setC = 5 + (fMax / c);
    
  dirA = (posA <= newA) ? 1 : -1;
  dirB = (posB <= newB) ? 1 : -1;
  dirC = (posC <= newC) ? 1 : -1;

  if( dirA > 0){
    mask |= (1<<0);
  }
  if( dirB > 0){
    mask |= (1<<1);
  }
  if( dirC > 0){
    mask |= (1<<4);
  }

  PORTF = mask;
  
  nowCoils = coils;
  setCoils();  
  isOnPoint = false;
  return mask;
}

uint8_t StepControl::raiseMask(){
  uint8_t mask = 0;

  if( cntA >= setA >> 1){
    mask |= 1<<5;
  }

  if( cntB >= setB >> 1){
    mask |= 1<<6;
  }

  if( cntC >= setC >> 1){
    mask |= 1<<7;
  }

  if( cntA ){
    --cntA;
  }
  if( cntB ){
    --cntB;
  }
  if( cntC ){
    --cntC;
  }
  return mask;
}

void StepControl::onRise(StepControl *control){    
  PORTF |= control->raiseMask();    
}

uint8_t StepControl::fallMask(){
  uint8_t mask = 0;

  if( !cntA & (posA!=newA)){
    posA += dirA;
    cntA  = setA;
    mask |= 1 << 5;
  }

  if( !cntB & (posB!=newB)){
    posB += dirB;
    cntB  = setB;
    mask |= 1 << 6;
  }

  if( !cntC & (posC!=newC)){
    posC += dirC;
    cntC  = setC;
    mask |= 1 << 7;
  }
  return mask;
}

void StepControl::onFall(StepControl *control){  
  uint8_t mask = control->fallMask();
  PORTF &= ~mask;
  
  control->isOnPoint = (mask == 0);  
}

uint8_t StepControl::getInputs(){
  uint8_t tmp = PIND & 0b00011111;
  tmp |= (PIND & 0x80)>>2;
  return tmp;
}

bool StepControl::initialize(){  
  TIMER_STEP_STOP;
  TIMER_INIT_START;
  
  if( initAxist(0) == false ) {
    return false;
  }
  if( !initAxist(1) ) {
    return false;
  }
  if( !initAxist(2) ) {
    return false;
  }
  this->goTo(dZeroA, dZeroB, dZeroC,0);  
  
  return true;
}

bool StepControl::initAxist(uint8_t num){
  uint8_t mask = 0;
  int16_t *cnt = 0;
  
  if( num == 0 ){
    this->goTo(-10000, posB>>1, posC>>1, 0);    
    cnt = &posA;
  } else if( num == 1 ){
    this->goTo(posA>>1, -10000, posC>>1, 0);
    cnt = &posB;
  } else if( num == 2 ){
    this->goTo(posA>>1, posB>>1, -10000, 0);
    cnt = &posC;
  }

  while((mask == 0) && ((*cnt)>-19500)){
    mask = getInputs() & ((1<<num) | (1<<(num+3)));    
  }

  if( mask == 0 ){
    return false;
  }

  proclaimPois(num, -50);  
  
  return true;
}