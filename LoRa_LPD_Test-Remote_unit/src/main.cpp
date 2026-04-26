#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include "ProtocolBAM.h"


/*~~~~~Hardware Definitions~~~~~*/

// These are hardware specific to the Heltec WiFi LoRa 32 V3
// Cite: https://resource.heltec.cn/download/WiFi_LoRa32_V3/HTIT-WB32LA(F)_V3_Schematic_Diagram.pdf
#define PRG_BUTTON 0
#define LORA_NSS_PIN 8
#define LORA_SCK_PIN 9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN 12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

//////////*~~~~~Global Variables~~~~~*/

NumberOfDevices_t deviceCount = 0;
SourceID_t connectedDevices[MAX_DEVICES];

// Message buffers for specific devices
struct MessageBuffer {
  bool available[5];           // Track which buffer slots are available
  Packet messages[5];          // Buffer to store up to 5 messages
  uint8_t count;              // Number of messages currently buffered
};

MessageBuffer msgBuffer52;     // Message buffer for device ID 52
MessageBuffer msgBuffer240;    // Message buffer for device ID 240


//////////*~~~~~Radio Configuration~~~~~*/

// Initialize SX1262 radio
// Make a custom SPI device because *of course* Heltec didn't use the default SPI pins
SPIClass spi(FSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0); // Defaults, works fine
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiSettings);

// Channel state tracking
enum ChannelMode { DOWNLINK_MODE, UPLINK_MODE };
volatile ChannelMode currentChannel = UPLINK_MODE;


///////*~~~~~Interrupt Handlers~~~~~*/
volatile bool receivedFlag = false;
volatile bool buttonFlag = false;

// This function should be called when a complete packet is received.
//  It is placed in RAM to avoid Flash usage errors
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void receiveISR(void) {
  // WARNING:  No Flash memory may be accessed from the IRQ handler: https://stackoverflow.com/a/58131720
  //  So don't call any functions or really do anything except change the flag
  receivedFlag = true;
}

// This function should be called when a complete packet is received.
//  It is placed in RAM to avoid Flash usage errors
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void buttonISR(void) {
  // WARNING:  No Flash memory may be accessed from the IRQ handler: https://stackoverflow.com/a/58131720
  //  So don't call any functions or really do anything except change the flag
  buttonFlag = true;
}


////////////~~~~~~~Local Prototypes~~~~~~~//
bool addDevice(SourceID_t deviceID);
bool isIDConnected(SourceID_t deviceID);
void FWDData(Packet receivedPacket);
void sendDataFWDFailureResponse(SourceID_t dstID);
void sendJoinResponse(SourceID_t dstID);
void sendJoinFailureResponse(SourceID_t dstID);
void sendScanResponse();

//////////*~~~~~Helper Functions~~~~~*/
void error_message(const char* message, int16_t state) {
  Serial.printf("ERROR!!! %s with error code %d\n", message, state);
  while(true); // loop forever
}

// Switch to uplink channel (for listening to devices)
bool switchToUplinkChannel() {
  if (currentChannel == UPLINK_MODE) return true;
  
  //Serial.println("Switching to uplink channel...");
  configureRadioChannel(UPLINK_FREQ, UPLINK_BW, UPLINK_SF);
  currentChannel = UPLINK_MODE;
  
  // Start continuous reception on uplink channel
  int16_t state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    // Failed to start uplink reception
    return false;
  }
  //Serial.println("Now listening on uplink channel");
  return true;
}

// Switch to downlink channel (for forwarding messages)
bool switchToDownlinkChannel() {
  if (currentChannel == DOWNLINK_MODE) return true;
  
  //Serial.println("Switching to downlink channel...");
  configureRadioChannel(DOWNLINK_FREQ, DOWNLINK_BW, DOWNLINK_SF);
  currentChannel = DOWNLINK_MODE;
  //Serial.println("Ready for downlink transmission");
  return true;
}

void configureRadioChannel(float freq, float bw, uint8_t sf) {
  int16_t state = radio.setFrequency(freq);
  if (state != RADIOLIB_ERR_NONE) {
    error_message("Failed to set frequency", state);
  }
  state = radio.setBandwidth(bw);
  if (state != RADIOLIB_ERR_NONE) {
    error_message("Failed to set bandwidth", state);
  }
  state = radio.setSpreadingFactor(sf);
  if (state != RADIOLIB_ERR_NONE) {
    error_message("Failed to set spreading factor", state);
  }
}

void resumeReception() {
  switchToUplinkChannel();
  int16_t state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
      error_message("Resuming reception failed", state);
    }
}

// Message buffer management functions
void removeMessage(MessageBuffer* buffer, int index) {
    if (!buffer->available[index]) {
        buffer->available[index] = true;
        buffer->count--;
        // Clear the message data (optional)
        // buffer->messages[index] = {};
    }
}

// Check if buffer has available slots
bool hasAvailableSlot(MessageBuffer* buffer) {
    for(int i = 0; i < 5; i++) {
        if(buffer->available[i]) {
            return true;
        }
    }
    return false;
}

// Add message to buffer, returns index if successful, -1 if buffer full
int addMessageToBuffer(MessageBuffer* buffer, Packet message) {
    for(int i = 0; i < 5; i++) {
        if(buffer->available[i]) {
            buffer->messages[i] = message;
            buffer->available[i] = false;
            buffer->count++;
            return i;
        }
    }
    return -1; // Buffer full
}

// Check if buffer has any messages
bool hasAvailableMessage(MessageBuffer* buffer) {
    return buffer->count > 0;
}

// Get first available message from buffer and remove it
Packet getMessageFromBuffer(MessageBuffer* buffer) {
    for(int i = 0; i < 5; i++) {
        if(!buffer->available[i]) {  // Message exists
            Packet message = buffer->messages[i];
            removeMessage(buffer, i);  // Remove the message
            return message;
        }
    }
    // Return empty packet if no message found (shouldn't happen if hasAvailableMessage was checked)
    Packet emptyPacket = {};
    return emptyPacket;
}

// Send data response with buffered message
void sendDataResponse(SourceID_t dstID, Packet message) {
    Packet dataResponse;
    dataResponse.sourceID = 0; // Gateway ID
    dataResponse.dstID = dstID;
    dataResponse.msgType = MSG_TYPE_DATA_RESPONSE;
    
    // Format: "1" + original message data for success
    dataResponse.data = "1" + message.data;
    dataResponse.length = dataResponse.data.length();
    
    switchToDownlinkChannel();
    String responsePacket = createStringPacket(dataResponse);
    int16_t state = radio.transmit(responsePacket);
    if (state != RADIOLIB_ERR_NONE) {
        // Failed to send data response
    }
    resumeReception();
}

// Send empty data response when no messages available
void sendEmptyDataResponse(SourceID_t dstID) {
    Packet dataResponse;
    dataResponse.sourceID = 0; // Gateway ID
    dataResponse.dstID = dstID;
    dataResponse.msgType = MSG_TYPE_DATA_RESPONSE;
    dataResponse.data = "0NO_DATA";  // "0" indicates no data available
    dataResponse.length = dataResponse.data.length();
    
    switchToDownlinkChannel();
    String responsePacket = createStringPacket(dataResponse);
    int16_t state = radio.transmit(responsePacket);
    if (state != RADIOLIB_ERR_NONE) {
        // Failed to send empty data response
    }
    resumeReception();
}

// Function to find and transmit buffered messages
bool transmitBufferedMessages(MessageBuffer* buffer, SourceID_t deviceID) {
    bool transmitted = false;
    for(int i = 0; i < 5; i++) {
        if (!buffer->available[i]) {  // Message exists
            // TODO: Add actual transmission logic here
            // Transmitting buffered message
            
            // Placeholder for transmission - replace with actual transmit code
            bool transmissionSuccess = true; // Replace with actual transmission attempt
            
            if (transmissionSuccess) {
                removeMessage(buffer, i);  // Remove on success
                // Message transmitted and removed
                transmitted = true;
            }
        }
    }
    return transmitted;
}

// TODO does gateway need this? doesnt need to wait for ACKs from devices, just forward data to server and hope for best
/*
Packet sendAndListenForResponse(Packet sendPacket, MsgType_t expectedResponse) {
  for(int attempt = 0; attempt < TIMEOUT_ATTEMPS; attempt++) {
      // Sending packet
      
      // Transmit packet
      switchToUplinkChannel();
      String packetString = createStringPacket(sendPacket);
      int16_t state = radio.transmit(packetString.c_str());
      if (state != RADIOLIB_ERR_NONE) {
          // Transmission failed
          continue;
      }
      
      // Start listening for response
      resumeReception();
      unsigned long startTime = millis();
      
      // Wait for response within listen window
      while(millis() - startTime < RESPONSE_LISTEN_WINDOW) {
          if(receivedFlag) {
              receivedFlag = false;
              
              String received_data;
              state = radio.readData(received_data);
              if (state == RADIOLIB_ERR_NONE) {
                  // Parse received packet
                  Packet receivedPacket = parseStringPacket(received_data);
                  

                  // check if DST is self or broadcast
                  if(receivedPacket.dstID == mySourceID || receivedPacket.dstID == 0xFF){
                    // Check if it's the expected response type
                    if(receivedPacket.msgType == expectedResponse) {
                        // Expected response received
                        return receivedPacket; // Success
                    } else if(receivedPacket.msgType == MSG_TYPE_ERROR){
                        Serial.printf("Error message received from source %d: %s\n", receivedPacket.sourceID, receivedPacket.data.c_str());
                        return receivedPacket;
                    }else {
                        // Wrong message type received
                    }
                  }
              }
              
              // Resume listening if wrong message type or read error
              resumeReception();  
          }
          delay(10); // Small delay to prevent busy waiting
      }
      
      // Attempt timed out
      delay(BACKOFF_TIME);
  }
  
  Serial.printf("All attempts failed - no response of type %d received\n", expectedResponse);
  return {0, 0, MSG_TYPE_ERROR, 0, String("No response received")};
}
*/
String createStringPacket(Packet packet) {
  // [source ID, DST ID, msgType, length, data]
  String message = "";
  
  // Zero-pad sourceID to 3 digits
  char sourceIdStr[4];
  sprintf(sourceIdStr, "%03u", packet.sourceID);
  message += String(sourceIdStr);
  
  // Zero-pad dstID to 3 digits  
  char dstIdStr[4];
  sprintf(dstIdStr, "%03u", packet.dstID);
  message += String(dstIdStr);
  
  // msgType as single digit (0-9)
  message += String(packet.msgType);
  
  // Zero-pad length to 3 digits
  char lengthStr[4];
  sprintf(lengthStr, "%03u", packet.length);
  message += String(lengthStr);
  
  // Data as-is (no padding needed)
  message += packet.data;
  
  return message;
}

Packet parseStringPacket(const String& packetString) {
  Packet packet;
  
  // Parse sourceID from positions 0-2 (3 digits)
  packet.sourceID = packetString.substring(0, SOURCE_ID_STRING_LENGTH).toInt();
  
  // Parse dstID from positions 3-5 (3 digits)  
  packet.dstID = packetString.substring(SOURCE_ID_STRING_LENGTH, SOURCE_ID_STRING_LENGTH + DST_ID_STRING_LENGTH).toInt();
  
  // Parse msgType from position 6 (1 digit)
  packet.msgType = static_cast<MsgType_t>(packetString.substring(SOURCE_ID_STRING_LENGTH + DST_ID_STRING_LENGTH, SOURCE_ID_STRING_LENGTH + DST_ID_STRING_LENGTH + MSG_TYPE_STRING_LENGTH).toInt());
  
  // Parse length from positions 7-9 (3 digits)
  packet.length = packetString.substring(SOURCE_ID_STRING_LENGTH + DST_ID_STRING_LENGTH + MSG_TYPE_STRING_LENGTH, SOURCE_ID_STRING_LENGTH + DST_ID_STRING_LENGTH + MSG_TYPE_STRING_LENGTH + DATA_LENGTH_STRING_LENGTH).toInt();
  
  // Parse data (remaining characters after position 10)
  packet.data = packetString.substring(SOURCE_ID_STRING_LENGTH + DST_ID_STRING_LENGTH + MSG_TYPE_STRING_LENGTH + DATA_LENGTH_STRING_LENGTH);
  
  return packet;
}

bool addDevice(SourceID_t deviceID) {
  if (deviceCount >= MAX_DEVICES) return false;
  connectedDevices[deviceCount] = deviceID;
  // Serial.print("Device ");
  // Serial.print(deviceID);
  // Serial.println(" added to connected devices list.");
  deviceCount++;
    return true;
}

// Find device
bool isIDConnected(SourceID_t deviceID) {
    for(int i = 0; i < deviceCount; i++) {
        if(connectedDevices[i] == deviceID) {
            return true;
        }
    }
    return false; // Not found
}


// response functions
void sendScanResponse() {
  switchToDownlinkChannel();
  Packet networkADVPacket;
  networkADVPacket.sourceID = 0x00;
  networkADVPacket.dstID = 0xFF;
  networkADVPacket.msgType = MSG_TYPE_NETWORK_SCAN_RESPONSE;
  networkADVPacket.length = 0;
  networkADVPacket.data = "";

  //add network name, number of devices, and device IDs to the advertisement packet
  networkADVPacket.data = NETWORK_NAME + String(deviceCount % 10); // Single digit (0-9) as per header definition
  for (int i = 0; i < deviceCount; i++) {
    char deviceIdStr[4]; // Buffer for 3 digits + null terminator  
    sprintf(deviceIdStr, "%03d", connectedDevices[i]); // Pad with leading zeros to 3 digits
    networkADVPacket.data += String(deviceIdStr);
  }
  networkADVPacket.length = networkADVPacket.data.length();
  // Serial.print("Network advertisement packet created: ");
  // Serial.println(networkADVPacket.data);

  //transmit
  String advPacket = createStringPacket(networkADVPacket);
  int16_t state = radio.transmit(advPacket);
  if (state == RADIOLIB_ERR_NONE) {
    //Serial.println("Network Scan Response sent successfully.");
  } else {
    Serial.print("Failed to send network advertisement. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}

void sendDataReceivedResponse(SourceID_t dstID) {
  Packet dataReceivedResponse;
  dataReceivedResponse.sourceID = 0; // Gateway ID
  dataReceivedResponse.dstID = dstID;
  dataReceivedResponse.msgType = MSG_TYPE_DATA_UPLINK_ACK;
  dataReceivedResponse.length = 0;
  dataReceivedResponse.data = "";
  switchToDownlinkChannel();
  String responsePacket = createStringPacket(dataReceivedResponse);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    // Data received response sent
  } else {
    Serial.print("Failed to send data received response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}

void sendJoinResponse(SourceID_t dstID) {
  Packet joinResponse;
  joinResponse.sourceID = 0; // Gateway ID
  joinResponse.dstID = dstID;
  joinResponse.msgType = MSG_TYPE_JOIN_RESPONSE;
  joinResponse.length = 1;
  joinResponse.data = "1"; //success
  switchToDownlinkChannel();
  String responsePacket = createStringPacket(joinResponse);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    // Join response sent
  } else {
    Serial.print("Failed to send join response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}

void sendJoinFailureResponse(SourceID_t dstID) {
  Packet joinFailureResponse;
  String errorMsg = "Join failed: Already Exists or Maximum number of devices reached";
  joinFailureResponse.sourceID = 0; // Gateway ID
  joinFailureResponse.dstID = dstID;
  joinFailureResponse.msgType = MSG_TYPE_JOIN_RESPONSE;
  joinFailureResponse.length = errorMsg.length() + 1;
  joinFailureResponse.data = "0" + errorMsg; // fail plus reason
  switchToDownlinkChannel();
  String responsePacket = createStringPacket(joinFailureResponse);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    // Join failure response sent
  } else {
    Serial.print("Failed to send join failure response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}


void sendUplinkACKResponse(SourceID_t dstID) {
  Packet uplinkACK;
  uplinkACK.sourceID = 0; // Gateway ID
  uplinkACK.dstID = dstID;
  uplinkACK.msgType = MSG_TYPE_DATA_UPLINK_ACK;
  uplinkACK.length = 1;
  uplinkACK.data = "1"; //success
  switchToDownlinkChannel();
  String responsePacket = createStringPacket(uplinkACK);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    // Uplink ACK sent
  } else {
    Serial.print("Failed to send uplink ACK response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}

void sendUplinkFailureResponse(SourceID_t dstID, String errorCode) {
  Packet uplinkFailureResponse;
  uplinkFailureResponse.sourceID = 0; // Gateway ID
  uplinkFailureResponse.dstID = dstID;
  uplinkFailureResponse.msgType = MSG_TYPE_DATA_UPLINK_ACK;
  uplinkFailureResponse.length = errorCode.length() + 1;
  uplinkFailureResponse.data = "0" + errorCode; // fail plus reason
  switchToDownlinkChannel();
  String responsePacket = createStringPacket(uplinkFailureResponse);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    // Uplink failure response sent
  } else {
    Serial.print("Failed to send uplink failure response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}

void sendUplinkSuccessResponse(SourceID_t dstID) {
  Packet uplinkSuccessResponse;
  uplinkSuccessResponse.sourceID = 0; // Gateway ID
  uplinkSuccessResponse.dstID = dstID;
  uplinkSuccessResponse.msgType = MSG_TYPE_DATA_UPLINK_ACK;
  uplinkSuccessResponse.length = 1;
  uplinkSuccessResponse.data = "1"; // success
  switchToDownlinkChannel();
  String responsePacket = createStringPacket(uplinkSuccessResponse);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    // Uplink success response sent
  } else {
    Serial.print("Failed to send uplink success response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}

// TODO ACK uplink and store in buffer

// void send(Packet receivedPacket) {
//   // Forward the data to the destination device if it is connected
//   if(isIDConnected(receivedPacket.dstID)) {
//     if(sendAndListenForResponse(receivedPacket, MSG_TYPE_DATA_RECEIVED)) {
//     } else {
//       Serial.println("Failed to forward data.");
//     }
//   } else {
//     Serial.printf("FWD Data Request Received. Destination ID %d not available. Sending failure response...", receivedPacket.dstID);
//     sendDataFWDFailureResponse(receivedPacket.sourceID);
//   }
//   resumeReception();
// }

// Listen functions






/*~~~~~Application~~~~~*/
void setup() {
  Serial.begin(115200);

  // Initialize message buffers
  for(int i = 0; i < 5; i++) {
    msgBuffer52.available[i] = true;
    msgBuffer240.available[i] = true;
  }
  msgBuffer52.count = 0;
  msgBuffer240.count = 0;
  // Message buffers initialized

  // Set up GPIO pin for "PRG" button and enable interrupts for it
  pinMode(PRG_BUTTON, INPUT);
  attachInterrupt(PRG_BUTTON, buttonISR, FALLING);

  // Set up SPI with our specific pins 
  spi.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  // Initialize radio with uplink channel parameters (always listening for devices)
  // carrier frequency:           UPLINK_FREQ
  // bandwidth:                   UPLINK_BW 
  // spreading factor:            UPLINK_SF
  // coding rate:                 5 (CR 4/5 for LoRaWAN)
  // sync word:                   0x34 (LoRaWAN sync word)
  // output power:                0 dBm
  // preamble length:             8 symbols (LoRaWAN preamble length)
  Serial.print("Initializing radio with uplink channel (listen for devices)...");
  int16_t state = radio.begin(UPLINK_FREQ, UPLINK_BW, UPLINK_SF, 5, 0x34, 10, 8);
  if (state != RADIOLIB_ERR_NONE) {
      error_message("Radio initializion failed", state);
  }

  // Current limit of 140 mA (max)
  state = radio.setCurrentLimit(140.0);
  if (state != RADIOLIB_ERR_NONE) {
      error_message("Current limit intialization failed", state);
  }

  // Hardware uses DIO2 on the SX1262 as an RF switch
  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
      error_message("DIO2 as RF switch intialization failed", state);
  }

  // LoRa explicit header mode is used for LoRaWAN
  state = radio.explicitHeader();
  if (state != RADIOLIB_ERR_NONE) {
      error_message("Explicit header intialization failed", state);
  }

  // LoRaWAN uses a two-byte CRC
  state = radio.setCRC(2);
  if (state != RADIOLIB_ERR_NONE) {
      error_message("CRC intialization failed", state);
  }
  Serial.println("Complete!");

  // set the function that will be called when a new packet is received
  radio.setDio1Action(receiveISR);

  // Start on uplink channel (always listening for devices)
  Serial.print("Beginning continuous reception on uplink channel...");    ////////////////TODO check for proper start and swap
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    error_message("Starting reception failed", state);
  }
  Serial.println("Gateway ready! Listening for device messages on uplink channel.");
}

void loop() {

  // Handle packet receptions
  if (receivedFlag) {
    receivedFlag = false;

    // you can receive data as an Arduino String
    String packet_data;
    int state = radio.readData(packet_data);

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      //Serial.println("Received packet!");
      Packet receivedPacket = parseStringPacket(packet_data);


      //~~~~~~~~~~~~Handle Scan Request
      if(receivedPacket.msgType == MSG_TYPE_SCAN) {
        Serial.printf("Scan request received from device %d. Sending scan response...\n", receivedPacket.sourceID);
        sendScanResponse();
      }

      // ~~~~~~~~~~~~Handle join request~~~~~~~~~~~~
      if(receivedPacket.msgType == MSG_TYPE_JOIN_REQUEST) {
        // if new device, add and send good join response, 
        // else if existing device or max devices reached, send failure response
        if(!isIDConnected(receivedPacket.sourceID)) {
          if(addDevice(receivedPacket.sourceID)) {
            Serial.print("Device ");
            Serial.print(receivedPacket.sourceID);
            Serial.println(" joined the network.");
            sendJoinResponse(receivedPacket.sourceID);
          } else {
            Serial.println("Failed to add device: exists or maximum number of devices reached.");
            sendJoinFailureResponse(receivedPacket.sourceID);
          }
        }
      }

      // ~~~~~~~~~~~~Handle data uplink request~~~~~~~~~~~~
      if(receivedPacket.msgType == MSG_TYPE_DATA_UPLINK) {
        // Check if destination is connected
        if(isIDConnected(receivedPacket.dstID)) {
          // Check which device and if buffer is available
          if(receivedPacket.dstID == 52) {
            if(hasAvailableSlot(&msgBuffer52)) {
              int bufferIndex = addMessageToBuffer(&msgBuffer52, receivedPacket);
              if(bufferIndex >= 0) {
                Serial.printf("Message buffered: %d -> 52\n", receivedPacket.sourceID);
                sendUplinkSuccessResponse(receivedPacket.sourceID);
              } else {
                Serial.printf("Buffer full: %d -> 52\n", receivedPacket.sourceID);
                sendUplinkFailureResponse(receivedPacket.sourceID, "BUFFER_FULL");
              }
            } else {
              Serial.printf("Buffer full: %d -> 52\n", receivedPacket.sourceID);
              sendUplinkFailureResponse(receivedPacket.sourceID, "BUFFER_FULL");
            }
          }
          else if(receivedPacket.dstID == 240) {
            if(hasAvailableSlot(&msgBuffer240)) {
              int bufferIndex = addMessageToBuffer(&msgBuffer240, receivedPacket);
              if(bufferIndex >= 0) {
                Serial.printf("Message buffered: %d -> 240\n", receivedPacket.sourceID);
                sendUplinkSuccessResponse(receivedPacket.sourceID);
              } else {
                Serial.printf("Buffer full: %d -> 240\n", receivedPacket.sourceID);
                sendUplinkFailureResponse(receivedPacket.sourceID, "BUFFER_FULL");
              }
            } else {
              Serial.printf("Buffer full: %d -> 240\n", receivedPacket.sourceID);
              sendUplinkFailureResponse(receivedPacket.sourceID, "BUFFER_FULL");
            }
          }
          else {
            Serial.printf("No buffer for device %d\n", receivedPacket.dstID);
            sendUplinkFailureResponse(receivedPacket.sourceID, "DST_NO_BUFFER");
          }
        }
        else {
          Serial.printf("Unknown destination: %d\n", receivedPacket.dstID);
          sendUplinkFailureResponse(receivedPacket.sourceID, "DST_UNKNOWN");
        }
      }
         
      //~~~~~~~~~~Handle Data request from device
      if(receivedPacket.msgType == MSG_TYPE_DATA_REQUEST) {
        // Check if requesting device has buffered messages
        if(receivedPacket.sourceID == 52) {
          if(hasAvailableMessage(&msgBuffer52)) {
            Packet message = getMessageFromBuffer(&msgBuffer52);
            Serial.printf("Data sent to device 52 (queue: %d)\n", msgBuffer52.count);
            sendDataResponse(receivedPacket.sourceID, message);
          } else {
            Serial.printf("No data for device 52\n");
            sendEmptyDataResponse(receivedPacket.sourceID);
          }
        } 
        else if(receivedPacket.sourceID == 240) {
          if(hasAvailableMessage(&msgBuffer240)) {
            Packet message = getMessageFromBuffer(&msgBuffer240);
            Serial.printf("Data sent to device 240 (queue: %d)\n", msgBuffer240.count);
            sendDataResponse(receivedPacket.sourceID, message);
          } else {
            Serial.printf("No data for device 240\n");
            sendEmptyDataResponse(receivedPacket.sourceID);
          }
        } 
        else {
          // Device doesn't have a buffer (not 52 or 240)
          Serial.printf("No buffer for device %d\n", receivedPacket.sourceID);
          sendEmptyDataResponse(receivedPacket.sourceID);
        }
      }

    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // timeout occurred while waiting for a packet
      Serial.println("timeout!");
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println("CRC error!");
    } else {
      // some other error occurred
      Serial.print("failed, code ");
      Serial.println(state);
    }

    resumeReception();
  }

  // Handle button presses to advertise the network
  if(buttonFlag) {
    buttonFlag = false;
    // transmit network advertisement
    Serial.print("Button pressed! Resetting Network...");
    deviceCount = 0; // reset connected devices list
    memset(connectedDevices, 0, sizeof(connectedDevices));

    // reset the receivedFlag status and resume receiving
    receivedFlag = false;
    resumeReception();
  
  }

  // If you want some actions to happen with a time delay, use this
  //static unsigned long next_time = millis();
  //if (millis() > next_time) {
    //next_time += 1000;

    // periodic actions here
  //}
}

