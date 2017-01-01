/*
 * usb.h
 *
 * Created: 23.12.2016 22:42:15
 *  Author: Sync
 */ 

#ifndef USB_H_
#define USB_H_

#include <avr/interrupt.h>

extern "C" void USB_GEN_vect(void) __attribute__ ((signal));
extern "C" void USB_COM_vect(void) __attribute__ ((signal));

#define GET_STATUS			  0
#define CLEAR_FEATURE		  1
#define SET_FEATURE			  3
#define SET_ADDRESS			  5
#define GET_DESCRIPTOR		6
#define SET_DESCRIPTOR		7
#define GET_CONFIGURATION	8
#define SET_CONFIGURATION	9
#define GET_INTERFACE		  10
#define SET_INTERFACE		  11

#define GET_DESCRIPTOR_DEVICE     0x01
#define GET_DESCRIPTOR_CONFIG     0x02
#define GET_DESCRIPTOR_STRING     0x03
#define GET_DESCRIPTOR_QUALIFER   0x06

typedef bool (*functptr)(void *);

#pragma pack(1)

union USB_Request_Header_Extend{
  uint8_t bData[8];
  struct Buf32{
    uint32_t l32a;
    uint32_t l32b;
  };
  struct Buf16{
    uint16_t l16a;
    uint16_t l16b;
    uint16_t l16c;
    uint16_t l16d;
  };
};

struct USB_Request_Header{
  uint8_t bmRequestType;   /**< Type of the request. */
  uint8_t bRequest;        /**< Request command code. */
  uint8_t wValueL;          /**< wValue parameter of the request. */
  uint8_t wValueH;          /**< wValue parameter of the request. */
  uint8_t wIndexL;          /**< wIndex parameter of the request. */
  uint8_t wIndexH;          /**< wIndex parameter of the request. */
  uint8_t wLengthL;         /**< Length of the data to transfer in bytes. */
  uint8_t wLengthH;         /**< Length of the data to transfer in bytes. */
  USB_Request_Header_Extend extend;
};

struct USB_Device_Descriptor{
  uint8_t   len;
  uint8_t   dtype;
  uint16_t  usbVersion;
  uint8_t	  deviceClass;
  uint8_t	  deviceSubClass;
  uint8_t	  deviceProtocol;
  uint8_t	  packetSize0;
  uint16_t	idVendor;
  uint16_t	idProduct;
  uint16_t	deviceVersion;
  uint8_t	  iManufacturer;
  uint8_t	  iProduct;
  uint8_t	  iSerialNumber;
  uint8_t	  bNumConfigurations;
};

struct USB_Device_Qualifer_Descriptor{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint8_t  bNumConfigurations;
  uint8_t  bReserved;
};

struct USB_Config_Descriptor{
	uint8_t	  len;			
	uint8_t	  dtype;			
	uint16_t  clen;			
	uint8_t	  numInterfaces;
	uint8_t	  config;
	uint8_t	  iconfig;
	uint8_t	  attributes;
	uint8_t   maxPower;
};

struct USB_Interface_Descriptor{
 uint8_t bLength; /* size of this descriptor in bytes */
 uint8_t bDescriptorType; /* INTERFACE descriptor type */
 uint8_t bInterfaceNumber; /* Number of interface */
 uint8_t bAlternateSetting; /* value to select alternate setting */
 uint8_t bNumEndpoints; /* Number of EP except EP 0 */
 uint8_t bInterfaceClass; /* Class code assigned by the USB */
 uint8_t bInterfaceSubClass; /* Sub-class code assigned by the USB */
 uint8_t bInterfaceProtocol; /* Protocol code assigned by the USB */
 uint8_t iInterface; /* Index of string descriptor */
};

struct USB_Endpoint_Descriptor{
  uint8_t  bLength;          /* Size of this descriptor in bytes */
  uint8_t  bDescriptorType;  /* ENDPOINT descriptor type */
  uint8_t  bEndpointAddress; /* Address of the endpoint */
  uint8_t  bmAttributes;     /* Endpoint’s attributes */
  uint16_t wMaxPacketSize;   /* Maximum packet size for this EP */
  uint8_t  bInterval;        /* Interval for polling EP in ms */
};

enum upType{CONTROL,ISOCHR,BULK,INT};
enum upDirection{OUT,IN};
enum upBank{ONE,TWO};
enum upSize {SIZE8, SIZE16, SIZE32, SIZE64, SIZE128, SIZE256, SIZE512};
enum epIsoSync{NO_SYNC, ASYNC, ADAPT, SYNC};
enum epIsoType{DATA, FEEDBACK, exFEEDBACK};

struct USB_Endpoint_Definition{ 
  uint8_t     init;
  upType      type;
  upDirection dir;
  upSize      size;
  upBank      bank;
  uint8_t     interval;
  epIsoSync   sync;
  epIsoType   iType;
};

#define nop() do { __asm__ __volatile__ ("nop"); } while (0)

#pragma pack()

class USB{   

  protected:
    USB_Request_Header              controlHeader;
    USB_Device_Descriptor           device;
    USB_Device_Qualifer_Descriptor  qDevice;
    USB_Config_Descriptor           config;
    USB_Interface_Descriptor        iface;

    inline void sof();
    static void eorst();    
    void ControlRequest();
    static bool initInternalEndpoint(uint8_t num, upType type, upDirection dir,upSize size, upBank bank);
    bool getStatus();
    bool clearFeature();
    bool setFeature();
    bool setAddres();
    bool getDescriptor();
    bool setDescriptor();
    bool getConfiguration();
    inline bool setConfiguration();
    bool getInterface();
    bool setInterface();
  private:    
    bool isEnum;
    bool useStr;
    uint8_t currentEndpoins;
    uint8_t activeEP;
    uint8_t sendBufPos;
    uint8_t stringCounter;
    uint8_t epCounter;

    wchar_t *strings[50];
    volatile functptr epEvents[7];
    USB_Endpoint_Descriptor epDescriptors[7];    
    USB_Endpoint_Definition epDefinitions[7];
    
    void initPLL(void);    
    void waitIn(void);
    void clearIn(void);
    void stall(void);    
    static void selectEndPoint(uint8_t num);    
    inline uint8_t getStrId();
    void initControl(void);
    void initConfiguration(bool busPower, bool remoteWakeup, uint16_t powerVal);
    void initInterface(void);
    void activateEndpoints(void);
  public:    
    void init(uint16_t vid, uint16_t pid, float usbVer, bool useString, bool busPower, bool remoteWakeup, uint16_t powerVal);
    bool initEndpoint(uint8_t num, USB_Endpoint_Definition def);    
    bool isEnumerate();
    void readBuf(uint8_t *buffer, uint8_t size);
    void writeBuf(const uint8_t *buffer, uint8_t bufSize, uint8_t maxSize);
    void registerCallback(uint8_t num, functptr funct);
    void clearStall(uint8_t num);

    static void onComEvent(USB *handle);
    static void onGenEvent(USB *handle);

    friend void USB_COM_vect( void );
    friend void USB_GEN_vect( void );
};




#endif /* USB_H_ */