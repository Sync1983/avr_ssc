/*
 * stepControl.cpp
 *
 * Created: 29.12.2016 22:44:01
 *  Author: Sync
 */ 

#include <avr/interrupt.h>
#include <stdlib.h>
#include "stepControl.h"

void StepControl::init(pinIO *DIR_A, pinIO *DIR_B, pinIO *DIR_C){  

  dirStepA = DIR_A;
  dirStepB = DIR_B;
  dirStepC = DIR_C;

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

uint8_t StepControl::goTo(int16_t A, int16_t B, int16_t C, uint8_t coils){    
  const float fMax = 200.0f;
  float a,b,c;
  uint8_t mask = 0;

  newA = A*2;
  newB = B*2;
  newC = C*2;

  a = abs(posA - newA);
  b = abs(posB - newB);
  c = abs(posC - newC);
  if( a > fMax ) a = fMax;
  if( b > fMax ) b = fMax;
  if( c > fMax ) c = fMax;
    
  cntA = setA = 3 + (fMax / a);
  cntB = setB = 3 + (fMax / b);
  cntC = setC = 3 + (fMax / c);
    
  dirA = (posA <= newA) ? 1 : -1;
  dirB = (posB <= newB) ? 1 : -1;
  dirC = (posC <= newC) ? 1 : -1;

  if( dirA > 0 ){
    mask |= 1<<0;
  }

  if( dirB > 0 ){
    mask |= 1<<1;
  }

  if( dirC > 0){
    mask |= 1<<2;
  }
  
  nowCoils = coils;
  setCoils();  
  return mask;
}

uint8_t StepControl::onRise(StepControl *control){  
  uint8_t mask = 0;

  if( control->cntA == (control->setA>>1) ){    
    mask |= 1 << 0;
  }
  if( control->cntB == (control->setB>>1) ){    
    mask |= 1 << 1;
  }
  if( control->cntC == (control->setC>>1) ){    
    mask |= 1 << 2;
  }

  if( control->cntA ){    
    control->cntA--;
  } 
  if( control->cntB ){    
    control->cntB--;
  }
  if( control->cntC ){    
    control->cntC--;
  }    
  return mask;
}

uint8_t StepControl::onFall(StepControl *control){  
  uint8_t doneCnt = 0;
  uint8_t mask = 0xFF;

  if(!control->cntA && (control->posA != control->newA) ){
    mask &= ~(1 << 0);                                  
    control->posA += control->dirA;
    control->cntA = control->setA;                      
    doneCnt++;                                          
  }                                                     
  if(!control->cntB && (control->posB != control->newB) ){
    mask &= ~(1 << 1);                                  
    control->posB += control->dirB;
    control->cntB = control->setB;                      
    doneCnt++;                                          
  }                                                     
  if(!control->cntC && (control->posC != control->newC) ){
    mask &= ~(1 << 2);    
    control->posC += control->dirC;
    control->cntC = control->setC;
    doneCnt++;
  } 
  
  if( doneCnt>=3 ){
    control->isOnPoint = true;
  } else {
    control->isOnPoint = false;
  }
  return mask;
}