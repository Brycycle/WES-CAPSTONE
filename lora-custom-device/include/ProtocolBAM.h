#include <cstdint>
#include <Arduino.h>
#include <cstdlib>   // for rand() and srand()
#include <ctime>
using namespace std;


#define NETWORK_NAME String("BAM!")
// LoRa channel configuration
#define UPLINK_FREQ 923.3
#define UPLINK_BW 125.0
#define UPLINK_SF 8
#define DOWNLINK_FREQ 915.2
#define DOWNLINK_BW 125.0
#define DOWNLINK_SF 9


#define RESPONSE_LISTEN_WINDOW 1000
#define TIMEOUT_ATTEMPS 3
#define BACKOFF_TIME (rand() % 1000) //random backoff time between 0 and 999 ms
#define MAX_DEVICES 9
#define SLEEP_TIME 15000     // 15 seconds sleep (main wake/scan cycle)
#define SCAN_TIME 1000        // 1 second awake/scanning



//Packet Structure
//datatypes
typedef uint8_t SourceID_t; 
typedef uint8_t DstID_t; 
typedef uint8_t DataLength_t; 
typedef uint8_t NumberOfDevices_t;
typedef String Data_t; 

enum MsgType_t : uint8_t {
    MSG_TYPE_NETWORK_SCAN_RESPONSE = 0, // gateway response to scan
    MSG_TYPE_JOIN_REQUEST = 1,
    MSG_TYPE_JOIN_RESPONSE = 2,
    MSG_TYPE_DATA_UPLINK = 3, // device request data forward
    MSG_TYPE_DATA_UPLINK_ACK = 4, // ACK uplink request
    MSG_TYPE_DATA_REQUEST = 5, // device asks for downlink
    MSG_TYPE_ERROR = 6,
    MSG_TYPE_SCAN = 7, // device wake up and scan for network
    MSG_TYPE_DATA_RESPONSE = 8 // gateway response to data request
};

typedef struct Packet {
  SourceID_t sourceID;
  DstID_t dstID;
  MsgType_t msgType;
  DataLength_t length;
  Data_t data;
} Packet;

//ADV packet structure
// [NETWORK NAME, number of devices, deviceID1, deviceID2, ..., deviceIDn]
//string "[NETWORK NAME][number of devices][deviceID1, deviceID2, ..., deviceIDn]"

// length of each field (bytes)
#define SOURCE_ID_DATA_LENGTH 1
#define DST_ID_DATA_LENGTH 1
#define MSG_TYPE_DATA_LENGTH 1
#define DATA_LENGTH_DATA_LENGTH 1
#define NUMBER_OF_DEVICES_DATA_LENGTH 1

// length of each field when in String form (bytes)
#define SOURCE_ID_STRING_LENGTH 3 // uint8_t converted to string with 3 digits (zero padded)
#define DST_ID_STRING_LENGTH 3 // uint8_t converted to string with 3 digits (zero padded)
#define MSG_TYPE_STRING_LENGTH 1 //only ever one char
#define DATA_LENGTH_STRING_LENGTH 3 // uint8_t converted to string with 3 digits (zero padded)
#define NUMBER_OF_DEVICES_STRING_LENGTH 1 // only ever one digit
#define NETWORK_NAME_STRING_LENGTH 4 // "BAM!" is 4 characters, only ever in string form

//function prototypes
void resumeReception();
void configureRadioChannel(float freq, float bw, uint8_t sf);
Packet sendAndListenForResponse(Packet sendPacket, MsgType_t expectedResponse);
void sendDataReceivedResponse(SourceID_t dstID);
String createStringPacket(Packet packet);
Packet parseStringPacket(const String& packetString);
bool switchToDownlinkChannel();
bool switchToUplinkChannel();
void error_message(const char* message, int16_t state);

//commonvariables

