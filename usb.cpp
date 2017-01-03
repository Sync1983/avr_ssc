/*
 * usb.cpp
 *
 * Created: 23.12.2016 22:42:26
 *  Author: Sync
 */ 
  #define F_CPU 16E6
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "usb.h"

void USB::initPLL(){
  PLLFRQ   = 0b01001010;     //Set PLL freq 96MHz and PLL USB divider 2
  PLLCSR  |= (1<<PINDIV);    //Set PLL div for 16MHz
  PLLCSR  |= (1<<PLLE);

  while( (PLLCSR & 0x01) == 0)
    ;
}

void USB::init(uint16_t vid, uint16_t pid, float usbVer, bool useString, bool busPower, bool remoteWakeup, uint16_t powerVal){
  cli();

  this->isEnum = false;
  this->initPLL();
  UHWCON   = 0b00000001;
  USBCON  |= (1<<OTGPADE);
  USBCON  |= (1<<USBE) | (1<<FRZCLK);
  USBCON &= ~(1<<FRZCLK);               // UNFREEZE;

  //_delay_ms(10);

  UDCON &= ~(1<<DETACH);                // ATTACH;

  UDIEN = (1<<EORSTE); 

  this->useStr = useString;
  float mul     = usbVer * 10;
  uint8_t hiVer = (uint8_t)usbVer;
  uint8_t loVer = (uint8_t)((mul - hiVer*10))<<4;

  this->device.len                    = sizeof(device);
  this->device.dtype                  = 0x01;
  this->device.usbVersion             = (hiVer<<8) | loVer;
  this->device.deviceClass            = 0x00;
  this->device.deviceSubClass         = 0x00;
  this->device.deviceProtocol         = 0x00;
  this->device.packetSize0            = 64;
  this->device.idVendor               = vid;
  this->device.idProduct              = pid;
  this->device.deviceVersion          = 0x0101;
  this->device.iManufacturer          = this->getStrId();
  this->device.iProduct               = this->getStrId();
  this->device.iSerialNumber          = this->getStrId();
  this->device.bNumConfigurations     = 1;

  qDevice.bLength             = sizeof(USB_Device_Qualifer_Descriptor);
  qDevice.bDescriptorType     = 0x06;  
  qDevice.bcdUSB              = (hiVer<<8)|loVer;
  qDevice.bDeviceClass        = 0x00;
  qDevice.bDeviceSubClass     = 0x00;
  qDevice.bDeviceProtocol     = 0x00;
  qDevice.bMaxPacketSize0     = 64;
  qDevice.bNumConfigurations  = 1;
  qDevice.bReserved           = 0;

  initConfiguration(busPower, remoteWakeup, powerVal);
  initInterface();

  memset(&epDescriptors,0, sizeof(USB_Endpoint_Descriptor) * 7);

  for(uint8_t i=0; i<7;i++){    
    epEvents[i] = NULL;
  }
  epCounter = 1;
  initInternalEndpoint(0, CONTROL,IN,SIZE64,ONE);
  sei();
}

void USB::initConfiguration(bool busPower, bool remoteWakeup, uint16_t powerVal){
  config.len              = sizeof(USB_Config_Descriptor);  
  config.dtype            = 0x02;
  config.clen             = config.len;			
  config.numInterfaces    = 0;
  config.config           = 1;
  config.iconfig          = getStrId();
  config.attributes       = 0x80 | (busPower?0:0x40) | (remoteWakeup?0x20:0);
  config.maxPower         = powerVal >> 1;
}

void USB::initInterface(void){
  iface.bLength           = sizeof(USB_Interface_Descriptor);
  iface.bDescriptorType   = 0x04;
  iface.bInterfaceNumber  = 0; 
  iface.bAlternateSetting = 0;
  iface.bNumEndpoints     = 0;
  iface.bInterfaceClass   = 0xFF;
  iface.bInterfaceSubClass= 0x00;
  iface.bInterfaceProtocol= 0xFF;
  iface.iInterface        = getStrId();

  config.numInterfaces ++;
  config.clen += iface.bLength;
}

bool USB::initEndpoint(uint8_t num, USB_Endpoint_Definition def){
  cli();
  if( (num==0) || (num>6)) {
    return false;
  }

  memcpy(&epDefinitions[num],&def,sizeof(USB_Endpoint_Definition));
  epDefinitions[num].init = 1;

  USB_Endpoint_Descriptor *ep = &epDescriptors[num];
  
  ep->bLength           = sizeof(USB_Endpoint_Descriptor);
  ep->bDescriptorType   = 0x05;
  ep->bEndpointAddress  = num | ((def.dir==IN)?0x80:0);
  ep->bmAttributes      = def.type & 0b11;
  ep->wMaxPacketSize    = 8 * (1<<def.size);
  ep->bInterval         = 0;

  if( def.type == INT ){
    ep->bInterval = def.interval;
  } else if( def.type == ISOCHR ){
    ep->bInterval = 1;
    ep->bmAttributes |= ((def.sync & 0b11) <<2) | ((def.iType & 0b11) <<4); 
  }

  iface.bNumEndpoints++;
  config.clen += ep->bLength;
  
  sei();
  return true;
}

void USB::activateEndpoints(void){
  for(uint8_t i=1;i<7;i++){
    USB_Endpoint_Definition ep = epDefinitions[i];
    if( ep.init != 0 ){
      initInternalEndpoint(i,ep.type,ep.dir,ep.size,ep.bank);
      if( ep.dir == OUT ){
        UEIENX |= (1<<RXOUTE);
      } else {
        UEIENX |= (1<< TXINE);
      }
    }
  }
}

bool  USB::isEnumerate(){
  return this->isEnum;
}

uint8_t  USB::getStrId(){
  return (this->useStr?++this->stringCounter:0);
}

inline void  sof(void){

}

void USB::eorst(){   
  initInternalEndpoint(0,CONTROL,OUT,SIZE64,ONE);
  selectEndPoint(0);
  UEIENX = (1 << RXSTPE);  
}

void USB::waitIn(void){
  while (!(UEINTX & (1<<TXINI))) 
    ;
}

void USB::clearIn(void){
  UEINTX &= ~((1<<FIFOCON) | (1<<RXOUTI) |(1<<TXINI)); 
  this->sendBufPos = 0;
}

void USB::stall(void){
  UECONX |= (1<<STALLRQ);
}

void USB::clearStall(uint8_t num){
  USB::selectEndPoint(num);
  UECONX |= (1 << STALLRQC);
}

void USB::readBuf(uint8_t *buffer, uint8_t size){
  while(size--){
    *(buffer++) = UEDATX;
  }
}

void USB::writeBuf(const uint8_t *buffer, uint8_t bufSize, uint8_t maxSize){  
  if( (this->sendBufPos + bufSize) > maxSize ) {
    return;
  }
  uint8_t b,pos = 0;  
  while( bufSize-- ){
    b = buffer[pos++];
    UEDATX = b;
    this->sendBufPos++;
  }
}

void USB::selectEndPoint(uint8_t num){
  UENUM  = ((num) & 0x0F);
}

bool USB::initInternalEndpoint(const uint8_t num, upType type, upDirection dir,upSize size, upBank bank){
  volatile uint8_t b1 = 0,
          b2 = 0,
          tp = (uint8_t) type,
          dr = (uint8_t) dir,
          sz = (uint8_t) size,
          bn = (uint8_t) bank;

  b1 = (tp<<6) | dr;
  b2 = (sz<<4) | (bn<<2);
  
  USB::selectEndPoint(num);  
  UECONX |= (1<<EPEN);
  
  UECFG1X = 0;
  UECFG0X = b1;
  UECFG1X = (1<<ALLOC) | b2; 
  
  return (UESTA0X & (1<<CFGOK))?true:false;
}

void USB::registerCallback(uint8_t num, functptr funct){
  if( (num ==0) | (num > 6) ){
    return;
  }

  epEvents[num] = funct;
}

void USB::registerControlCallback(functCtrlptr funct){
  this->extendControl = funct;
}

inline void USB::ControlRequest(){
  selectEndPoint(0);

  volatile uint8_t intStatus = UEINTX;  

  if( (intStatus & (1<<RXSTPI)) == 0 ){    
    return;
  }

  UEIENX &= ~(1<<RXSTPE);       
  uint16_t cnt = UEBCX;
  this->readBuf((uint8_t*)&this->controlHeader,(uint8_t)cnt);  
  UEINTX = ~( (1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI) );
  sei();

  if( (controlHeader.bmRequestType & 0x80) == 0x80 ) { waitIn(); } else { clearIn(); };    

  bool accepted = true;  
  switch(controlHeader.bRequest){
    case GET_STATUS:      
        accepted = this->getStatus();
      break;
    case CLEAR_FEATURE:
//        accepted = this->clearFeature();
        nop();
      break;
    case SET_FEATURE:
//        accepted = this->setFeature();
        nop();
      break;
    case SET_ADDRESS:
        waitIn();
        accepted = this->setAddres();
      break;
    case GET_DESCRIPTOR:
        accepted = this->getDescriptor();
      break;
    case SET_DESCRIPTOR:
    case GET_CONFIGURATION:
        accepted = false;
      break;
    case SET_CONFIGURATION:         
      accepted = this->setConfiguration();                 
      break;
    case GET_INTERFACE:
    case SET_INTERFACE:
        accepted = false;        
      break;
    default:
      if( this->extendControl != 0){
        accepted = extendControl(this, this->controlHeader);
      } else {
        accepted = false;
      }
  }
  if( accepted ){
    clearIn(); 
  } else {
    this->stall();
  }  
  selectEndPoint(0);
  UEIENX |= (1 << RXSTPE);  

}

void USB::onGenEvent(USB *handle){
  uint8_t state = UDINT;
  UDINT = 0;
  
  if( state & (1<<EORSTI) ){
    handle->eorst();
  }  
}

void USB::onComEvent(USB *handle){  
  uint8_t pos, mask;
  
  handle->activeEP = UEINT;
  handle->currentEndpoins = UENUM;
  handle->ControlRequest();  
 
  for(pos = 1;pos<7;pos++){
    mask = (1<<pos);
    if( ((handle->activeEP & mask) == mask) && (handle->epEvents[pos] != 0)) {
      USB::selectEndPoint(pos);
      handle->epEvents[pos](handle);
      handle->clearIn();      
    }
  }
  selectEndPoint(handle->currentEndpoins);
}

bool USB::getStatus(){
  uint8_t status[2] = {0,0};
  writeBuf((const uint8_t*)&status,2,controlHeader.wLengthL);
  return true;
}

bool USB::setAddres(){   
   UDADDR  = controlHeader.wValueL;
   UDADDR |= (1<<ADDEN);
   return true;
}

bool  USB::getDescriptor(){
  switch( controlHeader.wValueH ){
    case GET_DESCRIPTOR_DEVICE:
      writeBuf((const uint8_t*)&device,sizeof(USB_Device_Descriptor), this->controlHeader.wLengthL);
    break;
    case GET_DESCRIPTOR_CONFIG:
      writeBuf((const uint8_t*)&config, sizeof(USB_Config_Descriptor),    this->controlHeader.wLengthL);
      writeBuf((const uint8_t*)&iface,  sizeof(USB_Interface_Descriptor), this->controlHeader.wLengthL);
      for(uint8_t pos = 1; pos<7;pos++){
        if( epDescriptors[pos].bLength != 0){
          writeBuf((const uint8_t*)&epDescriptors[pos],  sizeof(USB_Endpoint_Descriptor), this->controlHeader.wLengthL);
        }
      }      
    break;
    case GET_DESCRIPTOR_STRING:      
      return false;
    break;
    case GET_DESCRIPTOR_QUALIFER:
      writeBuf((const uint8_t*)&qDevice,sizeof(USB_Device_Qualifer_Descriptor), controlHeader.wLengthL);            
    break;
    default:
      return false;
  }
  return true;
}

bool  setDescriptor(){
  return false;
}

bool  getConfiguration(){
  return false;
}

bool USB::setConfiguration(){
  activateEndpoints();
  this->isEnum = true;
  return true;
}

bool  getInterface(){
  return false;
}

bool  setInterface(){
  return false;
}
