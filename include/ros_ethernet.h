#define STM32ETHERNET
#define ROSSERIAL_ARDUINO_TCP
#include <SPI.h>
#include <LwIP.h>
#include <STM32Ethernet.h>


// Set the shield settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);

// Set the rosserial socket server IP address
IPAddress server(192, 168, 1, 195);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;
