#ifndef PIN_CONTROL_
#define PIN_CONTROL_
#include <avr/io.h>

class pinIO{

  public:
    pinIO(volatile uint8_t* PORT, uint8_t PIN);
    void Set();
    void Clr();
    void Toggle();

  private:
    volatile uint8_t* port;
    volatile uint8_t pin;
};

#endif
