#include <cstdint>
#include <Arduino.h>
#include <cstdlib>   // for rand() and srand()
#include <ctime>
using namespace std;


// LoRa channel configuration //Beacon Tx on uplink, remote unit Tx on downlink
#define TX_FREQ 914.9
#define TX_BW 125.0       // 10.4, 125, 500
#define TX_SF 12           // 7, 12
#define ACK_FREQ 923.3
#define ACK_BW 125.0       // 10.4, 125, 500
#define ACK_SF 12           // 7, 12


#define RESPONSE_LISTEN_WINDOW 500 //250 for SF 7, 500 for SF 12

#define TEST_PACKET_50B "TESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKET" //50 byte (400bit) packet for testing
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
