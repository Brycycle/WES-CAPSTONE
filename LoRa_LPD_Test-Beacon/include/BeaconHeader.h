#include <cstdint>
#include <Arduino.h>
#include <cstdlib>   // for rand() and srand()
#include <ctime>
using namespace std;

// Target minimum: 50B payload = 400bits + overhead = ~450bps
/*
  SF 7 , BW 10.4   DReff ~455 bps
  SF 7 , BW 25     DReff ~1000 bps
  SF 7 , BW 125    DReff 5468 bps
  SF 7 , BW 500    DReff 21,875 bps

  SF 12, BW 10.4   DReff 24 bps       questionable for time?
  SF 12, BW 125    DReff 292.96 bps   under target bps? - still do it
  SD 12, BW 500    DReff 1171 bps
*/ 

/*
  SF 7 , BW 10.4   750 works with break
  SF 7 , BW 25     750 works with break (faster than 750)
  SF 7 , BW 125    500 works with break (faster than 500)
  SF 7 , BW 500    500 works with break (faster than 500)

  SF 12, BW 10.4   15000 with break const CRC error at remote unit on Rx, 
                    no ack. not doing?
  SF 12, BW 125    1500 works with break
  SD 12, BW 500    500 works with break



*/

#define TEST_SF 12          // 7, 12
#define TEST_BW_INT 500    // Integer for preprocessor comparisons: 10, 25, 125, 500
#define TEST_BW 500.0      // Float for actual radio config: 10.4, 25.0, 125.0, 500.0
#define OUTPUT_POWER 1


// LoRa channel configuration //Beacon Tx on uplink, remote unit Tx on downlink
#define TX_FREQ 914.9
#define TX_BW TEST_BW       
#define TX_SF TEST_SF           
#define ACK_FREQ 923.3
#define ACK_BW TEST_BW       // 10.4, 125, 500
#define ACK_SF TEST_SF         

// Dynamic RESPONSE_LISTEN_WINDOW based on SF and BW configuration for Tx of ACK of 10Bytes
#if TEST_SF == 7
  #if TEST_BW_INT == 10
    #define RESPONSE_LISTEN_WINDOW 750   //1000 works, 500 no, 750 works
  #elif TEST_BW_INT == 20
    #define RESPONSE_LISTEN_WINDOW 750  // 750 works, is success faster than 750
  #elif TEST_BW_INT == 125
    #define RESPONSE_LISTEN_WINDOW 500  //250 works, is it too fast? success is much faster than 250ms
  #elif TEST_BW_INT == 500              //making 500 min
    #define RESPONSE_LISTEN_WINDOW 500  // same above
  #else
    #define RESPONSE_LISTEN_WINDOW 1000   
  #endif
#elif TEST_SF == 12
  #if TEST_BW_INT == 10
    #define RESPONSE_LISTEN_WINDOW 15000 // const CRC error at remote unit on Rx, no ack.
  #elif TEST_BW_INT == 125
    #define RESPONSE_LISTEN_WINDOW 1500  // 1000 remote apears to Rx and ACK but SUT no ACK. 1500 works.
  #elif TEST_BW_INT == 500
    #define RESPONSE_LISTEN_WINDOW 500   //works
  #else
    #define RESPONSE_LISTEN_WINDOW 20000  
  #endif
#else
  #define RESPONSE_LISTEN_WINDOW 1000     
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
