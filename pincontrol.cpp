#include "pincontrol.h"

pinIO::pinIO(volatile uint8_t* PORT, uint8_t PIN){
  this->port = PORT;
  this->pin  = PIN;
}

void pinIO::Set(){
  *port |= (1<<pin);
}

void pinIO::Clr(){
  *port &= ~(1<<pin);
}

void pinIO::Toggle(){
  *port ^= (1<<pin);  
}
