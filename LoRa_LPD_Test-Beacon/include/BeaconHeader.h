#include <cstdint>
#include <Arduino.h>
#include <cstdlib>   // for rand() and srand()
#include <ctime>
using namespace std;

#define TEST_SF 12
#define TEST_BW_INT 500    // Integer for preprocessor comparisons
#define TEST_BW 500.0      // Float for actual radio config


// LoRa channel configuration //Beacon Tx on uplink, remote unit Tx on downlink
#define TX_FREQ 914.9
#define TX_BW TEST_BW       // 10.4, 125, 500
#define TX_SF TEST_SF           // 7, 12
#define ACK_FREQ 923.3
#define ACK_BW TEST_BW       // 10.4, 125, 500
#define ACK_SF TEST_SF           // 7, 12

// Dynamic RESPONSE_LISTEN_WINDOW based on SF and BW configuration
#if TEST_SF == 7
  #if TEST_BW_INT == 10
    #define RESPONSE_LISTEN_WINDOW 2000   
  #elif TEST_BW_INT == 125
    #define RESPONSE_LISTEN_WINDOW 250   
  #elif TEST_BW_INT == 500
    #define RESPONSE_LISTEN_WINDOW 250   
  #else
    #define RESPONSE_LISTEN_WINDOW 2000   
  #endif
#elif TEST_SF == 12
  #if TEST_BW_INT == 10
    #define RESPONSE_LISTEN_WINDOW 90000 
  #elif TEST_BW_INT == 125
    #define RESPONSE_LISTEN_WINDOW 5000  
  #elif TEST_BW_INT == 500
    #define RESPONSE_LISTEN_WINDOW 1500   
  #else
    #define RESPONSE_LISTEN_WINDOW 90000  
  #endif
#else
  #define RESPONSE_LISTEN_WINDOW 250     
#endif

#define TEST_PACKET_50B "!@#$%^&*()_+1234567890-=`~[{]}|;:',<.>/?qQwWeErRtT" //50 byte (400bit) packet for testing
#define TEST_PACKET_125B  "!@#$%^&*()_+1234567890-=`~[{]}|;:',<.>/?qQwWeErRtTyYuUiIoOpPaAsSdDfFgGhHjJkKlLzZxXcCvVbBnNmMñÑ¿¡áÁéÉíÍóÓúÚ£╛╜╧⌐╕ªº«»äëïöüÄ╙╪Ö" //125 byte (1000bit) packet for testing 


//function prototypes
void resumeReception();
void configureRadioChannel(float freq, float bw, uint8_t sf);
void error_message(const char* message, int16_t errorCode);
void TXandListenforACK();
void generateandTXACK(String packet_data);
double calcBER(String packet_data);

//common variables
extern bool TXToggle;
