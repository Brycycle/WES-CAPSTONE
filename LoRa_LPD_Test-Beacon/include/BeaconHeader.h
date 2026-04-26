#include <cstdint>
#include <Arduino.h>
#include <cstdlib>   // for rand() and srand()
#include <ctime>
using namespace std;


// LoRa channel configuration //Beacon Tx on uplink, remote unit Tx on downlink
#define FREQ 923.3
#define BW 125.0       // 10.4, 125, 500
#define SF 7           // 7, 12



#define RESPONSE_LISTEN_WINDOW 1000

#define TEST_PACKET_10B "TESTPACKET" //10 byte (80bit) packet for testing
#define TEST_PACKET_50B "TESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKET" //50 byte (400bit) packet for testing
#define TEST_PACKET_100B "TESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKET" //100 byte (800bit) packet for testing
#define TEST_PACKET_200B  "TESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKET"
                          "TESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKETTESTPACKET" //200 byte packet for testing


//function prototypes
void resumeReception();
void configureRadioChannel(float freq, float bw, uint8_t sf);
void error_message(const char* message, int16_t errorCode);
void TXandListenforACK();
void generateandTXACK(String packet_data);
double calcBER(String packet_data);

//common variables
extern bool TXToggle;

