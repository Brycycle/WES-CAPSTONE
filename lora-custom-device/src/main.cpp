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


/*~~~~~Global Variables~~~~~*/
SourceID_t mySourceID;
bool inNetwork = false;
String currentNetworkName = "";
SourceID_t connectedDevices[MAX_DEVICES];
NumberOfDevices_t deviceCount = 0;
bool scanResponseReceived = false;
bool dataResponseReceived = false;
// Periodic Wake/Scan/Sleep Cycle - Independent of button press
static unsigned long cycle_state_time = millis();
enum WakeScanCycle { CYCLE_WAKING, CYCLE_SCANNING, CYCLE_SCAN_LISTENING, 
                      CYCLE_DATA_REQUEST, CYCLE_DATA_LISTENING, CYCLE_SLEEPING };
static WakeScanCycle wakeScanState = CYCLE_SLEEPING;
static bool sleepStateEntered = false;

enum DeviceState { SLEEPING, SCANNING };
static DeviceState deviceState = SCANNING;  // Start awake



/*~~~~~Radio Configuration~~~~~*/

// Initialize SX1262 radio
// Make a custom SPI device because *of course* Heltec didn't use the default SPI pins
SPIClass spi(FSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0); // Defaults, works fine
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiSettings);

// Channel state tracking
enum ChannelMode { DOWNLINK_MODE, UPLINK_MODE };
volatile ChannelMode currentChannel = UPLINK_MODE;


/*~~~~~Function Prototypes~~~~~*/
void error_message(const char* message, int16_t state);
bool switchToDownlinkChannel();
bool switchToUplinkChannel();
void configureRadioChannel(float freq, float bw, uint8_t sf);


/*~~~~~Interrupt Handlers~~~~~*/
volatile bool receivedFlag = false;
volatile bool buttonFlag = false;
static bool downlinkRxActive = false;
static bool radioSleeping = false;

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


////////////*~~~~~Helper Functions~~~~~*/
void error_message(const char* message, int16_t state) {
  Serial.printf("ERROR!!! %s with error code %d\n", message, state);
  while(true); // loop forever
}

// Switch to uplink channel (for for transmitting to Gateway)
bool switchToUplinkChannel() {
  if (currentChannel == UPLINK_MODE) return true;
  
  //Serial.println("Switching to uplink channel...");
  configureRadioChannel(UPLINK_FREQ, UPLINK_BW, UPLINK_SF);
  currentChannel = UPLINK_MODE;
  downlinkRxActive = false;
  //Serial.println("Ready for uplink transmission");
  return true;
}

// Switch to downlink channel (for listening for activity)
bool switchToDownlinkChannel() {
  if (currentChannel == DOWNLINK_MODE) return true;
  
  //Serial.println("Switching to downlink channel...");
  configureRadioChannel(DOWNLINK_FREQ, DOWNLINK_BW, DOWNLINK_SF);
  currentChannel = DOWNLINK_MODE;
  downlinkRxActive = false;
  //Serial.println("Now listening on downlink channel");
  return true;

}

void configureRadioChannel(float freq, float bw, uint8_t sf) {
  // Wake radio from sleep mode if needed before configuration
  int16_t state = radio.standby();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Warning: Failed to wake radio from sleep: %d\n", state);
  } else {
    radioSleeping = false;
  }
  
  state = radio.setFrequency(freq);
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
  // Avoid re-entering RX repeatedly when already listening on downlink.
  if ((currentChannel == DOWNLINK_MODE) && downlinkRxActive) {
    return;
  }

  // If radio is asleep, wake it before attempting to start RX.
  if (radioSleeping) {
    int16_t wakeState = radio.standby();
    if (wakeState != RADIOLIB_ERR_NONE) {
      error_message("Wake before resume reception failed", wakeState);
    }
    radioSleeping = false;
  }

  if (!switchToDownlinkChannel()) {
    error_message("Resuming reception channel switch failed", -1);
  }

  int16_t state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    error_message("Resuming reception failed", state);
  }

  downlinkRxActive = true;
}

Packet sendAndListenForResponse(Packet sendPacket, MsgType_t expectedResponse) {
  for(int attempt = 0; attempt < TIMEOUT_ATTEMPS; attempt++) {
      
      // Transmit packet
      switchToUplinkChannel();
      String packetString = createStringPacket(sendPacket);
      int16_t state = radio.transmit(packetString.c_str());
      if (state != RADIOLIB_ERR_NONE) {
          Serial.printf("Transmission failed: %d\n", state);
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
                        return receivedPacket; // Success
                    } else if(receivedPacket.msgType == MSG_TYPE_ERROR){
                        Serial.printf("Error message received from source %d: %s\n", receivedPacket.sourceID, receivedPacket.data.c_str());
                        return receivedPacket;
                    }
                  }
              }
              
              // Resume listening if wrong message type or read error
              resumeReception();  
          }
          delay(10); // Small delay to prevent busy waiting
      }
      
      // Timeout, continue to next attempt
      delay(BACKOFF_TIME);
  }
  
  // All attempts failed
  return {0, 0, MSG_TYPE_ERROR, 0, String("No response received")};
}

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

uint8_t generateUniqueSourceID() {
    uint64_t chipId = ESP.getEfuseMac();
    return (uint8_t)(chipId & 0xFF); // Use lowest byte
}

bool addDevice(SourceID_t deviceID) {
  if (deviceID == mySourceID) return false;
  if (deviceCount >= MAX_DEVICES) return false;
  for (int i = 0; i < deviceCount; i++) {
    if (connectedDevices[i] == deviceID) return false;
  }
  connectedDevices[deviceCount] = deviceID;
  deviceCount++;
  return true;
}

void sendScanRequest() {
  Packet scanPacket;
  scanPacket.sourceID = mySourceID;
  scanPacket.dstID = 0x00; // Gateway ID
  scanPacket.msgType = MSG_TYPE_SCAN;
  scanPacket.length = 0;
  scanPacket.data = "";
  switchToUplinkChannel();
  String packetString = createStringPacket(scanPacket);
  int16_t state = radio.transmit(packetString);
  if (state != RADIOLIB_ERR_NONE) {
    // Scan request failed, will retry next cycle
  }
  resumeReception();
}

//parses an ADV packet to get other devices on the network and add to the connectedDevices list
void parseScanResponse(Packet packet) {
  // Check if the packet is a network scan response
  if (packet.msgType != MSG_TYPE_NETWORK_SCAN_RESPONSE) return;

  // Reset all local network state so this packet becomes the source of truth.
  deviceCount = 0;
  memset(connectedDevices, 0, sizeof(connectedDevices));
  inNetwork = false;

  const int idsStartPos = NETWORK_NAME_STRING_LENGTH + NUMBER_OF_DEVICES_STRING_LENGTH;
  if (packet.data.length() < idsStartPos) {
    return;
  }

  String networkName = packet.data.substring(0, NETWORK_NAME_STRING_LENGTH);
  currentNetworkName = networkName;
  
  int advertisedDeviceCount = packet.data.substring(NETWORK_NAME_STRING_LENGTH, NETWORK_NAME_STRING_LENGTH + NUMBER_OF_DEVICES_STRING_LENGTH).toInt();

  // Parse only complete IDs that are actually present in payload.
  int bytesForIds = packet.data.length() - idsStartPos;
  int availableDeviceIds = bytesForIds / SOURCE_ID_STRING_LENGTH;
  int parsedDeviceCount = advertisedDeviceCount;
  if (parsedDeviceCount > availableDeviceIds) {
    parsedDeviceCount = availableDeviceIds;
  }

  for (int i = 0; i < parsedDeviceCount; i++) {
    int deviceIdPos = idsStartPos + (i * SOURCE_ID_STRING_LENGTH);
    SourceID_t deviceID = packet.data.substring(deviceIdPos, deviceIdPos + SOURCE_ID_STRING_LENGTH).toInt();

    if (deviceID == mySourceID) {
      inNetwork = true;
    } else {
      addDevice(deviceID);
    }
  }

  Serial.printf("SCAN-RSP net=%s numIDs=%d peers=%d\n", networkName.c_str(), parsedDeviceCount, deviceCount);
}

/*
void sendDataReceivedResponse(SourceID_t dstID) {
  Packet dataReceivedResponse;
  dataReceivedResponse.sourceID = mySourceID; 
  dataReceivedResponse.dstID = dstID;
  dataReceivedResponse.msgType = MSG_TYPE_DATA_RECEIVED;
  dataReceivedResponse.length = 0;
  dataReceivedResponse.data = "";
  switchToUplinkChannel();
  String responsePacket = createStringPacket(dataReceivedResponse);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Data received response sent successfully.");
  } else {
    Serial.print("Failed to send data received response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}
*/

void sendJoinRequest() {
  Packet joinRequest;
  joinRequest.sourceID = mySourceID;
  joinRequest.dstID = 0x00; // Gateway ID
  joinRequest.msgType = MSG_TYPE_JOIN_REQUEST;
  joinRequest.length = 0;
  joinRequest.data = "";
  Serial.println("Attemping to JOIN Network...");
  Packet joinResponse = sendAndListenForResponse(joinRequest, MSG_TYPE_JOIN_RESPONSE);
  if(joinResponse.msgType == MSG_TYPE_JOIN_RESPONSE && joinResponse.data[0] == '1') {
    inNetwork = true;
    Serial.printf("JOIN ok net=%s\n", currentNetworkName.length() ? currentNetworkName.c_str() : "UNKNOWN");
  } else if (joinResponse.msgType == MSG_TYPE_JOIN_RESPONSE && joinResponse.data[0] == '0') {
    inNetwork = false;
    Serial.println("Join request rejected by gateway. Error: " + joinResponse.data.substring(1));
  }
  
  resumeReception();
}

void sendDataRequest(){
  Packet dataRequest;
  dataRequest.sourceID = mySourceID;
  dataRequest.dstID = 0x00; // Gateway ID
  dataRequest.msgType = MSG_TYPE_DATA_REQUEST;
  dataRequest.length = 0;
  dataRequest.data = "";
  
  switchToUplinkChannel();
  String packetString = createStringPacket(dataRequest);
  int16_t state = radio.transmit(packetString.c_str());
  if (state == RADIOLIB_ERR_NONE) {
    //Serial.println("DATA-REQUEST sent");
  } else {
    // Data request failed, will retry next cycle
  }
  resumeReception();
}

/*
void sendDataReceivedResponse(SourceID_t dstID) {
  Packet dataReceivedResponse;
  dataReceivedResponse.sourceID = mySourceID; 
  dataReceivedResponse.dstID = dstID;
  dataReceivedResponse.msgType = MSG_TYPE_DATA_RECEIVED;
  dataReceivedResponse.length = 0;
  dataReceivedResponse.data = "";
  switchToUplinkChannel();
  String responsePacket = createStringPacket(dataReceivedResponse);
  int16_t state = radio.transmit(responsePacket);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Data received response sent successfully.");
  } else {
    Serial.print("Failed to send data received response. Error code: ");
    Serial.println(state);
  }
  resumeReception();
}
*/

// ~~~~~~~~~~~~~~TODO
void sendDataUplink(){
  // Debug: Check if we have any other devices to send to
  if (deviceCount == 0) {
    Serial.println("No devices in network");
    return;
  }
  
  Serial.printf("Sending data to device %d\n", connectedDevices[0]);

  String dataToSend = "Hello from device " + String(mySourceID);
  
  Packet uplinkRequest;
  uplinkRequest.sourceID = mySourceID;
  uplinkRequest.dstID = connectedDevices[0];
  uplinkRequest.msgType = MSG_TYPE_DATA_UPLINK;
  uplinkRequest.length = dataToSend.length();
  uplinkRequest.data = dataToSend;
  Packet uplinkResponse = sendAndListenForResponse(uplinkRequest, MSG_TYPE_DATA_UPLINK_ACK);
  if(uplinkResponse.msgType == MSG_TYPE_DATA_UPLINK_ACK && uplinkResponse.data[0] == '1') {
    Serial.print("Data uplink request accepted by gateway. ");
  } else if (uplinkResponse.msgType == MSG_TYPE_DATA_UPLINK_ACK && uplinkResponse.data[0] == '0') {
    Serial.println("Data uplink request rejected by gateway. Error: " + uplinkResponse.data.substring(1));
  }
  
  resumeReception();
}


/*~~~~~Application~~~~~*/
void setup() {
  Serial.begin(115200);

  // Set up GPIO pin for "PRG" button and enable interrupts for it
  pinMode(PRG_BUTTON, INPUT);
  attachInterrupt(PRG_BUTTON, buttonISR, FALLING);

  // Set up SPI with our specific pins 
  spi.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  // Initialize radio with downlink channel parameters (always listening for activity)
  // carrier frequency:           DOWNLINK_FREQ
  // bandwidth:                   DOWNLINK_BW 
  // spreading factor:            DOWNLINK_SF
  // coding rate:                 5 (CR 4/5 for LoRaWAN)
  // sync word:                   0x34 (LoRaWAN sync word)
  // output power:                0 dBm
  // preamble length:             8 symbols (LoRaWAN preamble length)
  Serial.print("Initializing radio with downlink channel (listen for activity)...");
  int16_t state = radio.begin(DOWNLINK_FREQ, DOWNLINK_BW, DOWNLINK_SF, 5, 0x34, 10, 8);
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

  // Get source ID
  mySourceID = generateUniqueSourceID();
  Serial.print("Generated unique ID: ");
  Serial.println(mySourceID);

  // Initialize random seed using unique device ID to ensure different timing per device
  srand(mySourceID + millis());
  Serial.println("Random seed initialized for cycle timing variation");

  // Start on downlink channel (always listening for activity)
  // start continuous reception
  resumeReception();
  Serial.println("Complete!");
}

void loop() {

  // Handle packet receptions
  if (receivedFlag) {
    receivedFlag = false;
    downlinkRxActive = false;

    // you can receive data as an Arduino String
    String packet_data;
    int state = radio.readData(packet_data);

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      //Serial.println("Received packet!");
      Packet receivedPacket = parseStringPacket(packet_data);

      //Handle packets addressed to this device or broadcast (from gateway)
      if((receivedPacket.dstID == mySourceID) || (receivedPacket.dstID == 0xFF)){  
        
        // ~~~~~~~~~~~~Handle Gateway SCAN RESPONSE~~~~~~~~~~~~
        if(receivedPacket.msgType == MSG_TYPE_NETWORK_SCAN_RESPONSE) {
          //Serial.print("Received network scan response from gateway: ");
          parseScanResponse(receivedPacket);
          scanResponseReceived = true;  // Set flag to indicate successful scan response
        }

        // ~~~~~~~~~~~~Handle Gateway DATA RESPONSE~~~~~~~~~~~~
        if(receivedPacket.msgType == MSG_TYPE_DATA_RESPONSE) {
          if(receivedPacket.data[0] == '1') {
            Serial.println("Data available");
            Serial.println("Data from gateway: " + receivedPacket.data.substring(1));
          } else {
            Serial.println("No data");
          }
          dataResponseReceived = true;  // Set flag to indicate successful data response
        }

       
      } else{
        //Serial.println("Packet not relevant to self.");
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
    bool wasSleeping = (wakeScanState == CYCLE_SLEEPING);
    
    //~~~~~~~~~~~~ netowrk request (if not in network)
    if(!inNetwork) {
      sendJoinRequest();

    //~~~~~~~~~~~~ Send data to other device (if in network and another device is available)  
    } 
    

    //~~~~~~~~~~~~ Send data to other device (if in network and another device is available)  
    else if(inNetwork && (deviceCount != 0)) {
      sendDataUplink();
      //Serial.println("Data uplink request to gateway not implemented yet. Would send data to device ID: " + String(connectedDevices[0]));
      
    } else {
      Serial.println("No other devices available in network..");
    }

    // Reset reception flag and restore radio mode to what cycle state was active before button press.
    receivedFlag = false;
    if (wasSleeping) {
      // If already asleep, don't issue sleep command again (causes SPI timeout -705).
      if (!radioSleeping) {
        int16_t sleepState = radio.sleep();
        if (sleepState != RADIOLIB_ERR_NONE) {
          error_message("Restore sleep mode failed", sleepState);
        }
      }
      radioSleeping = true;
      downlinkRxActive = false;
    } else {
      resumeReception();
    }
  }

 
  
  
  switch(wakeScanState) {
    case CYCLE_WAKING:
      // Step 1: Wake up from sleep
      //Serial.println("Cycle: Waking up from sleep...");
      // Explicitly wake SX1262 from sleep before re-entering RX mode.
      {
        int16_t wakeState = radio.standby();
        if (wakeState != RADIOLIB_ERR_NONE) {
          error_message("Wake from sleep failed", wakeState);
        }
        radioSleeping = false;
      }
      resumeReception();
      
      wakeScanState = CYCLE_SCANNING;
      cycle_state_time = millis();
      break;
      
    case CYCLE_SCANNING:
      // Step 2: Send Scan on uplink
        Serial.println("Scanning");
      scanResponseReceived = false;  // Reset before TX to avoid dropping a fast valid response
      sendScanRequest();

      wakeScanState = CYCLE_SCAN_LISTENING;
      cycle_state_time = millis();
      break;
      
    case CYCLE_SCAN_LISTENING:
      if (scanResponseReceived) {
        //Serial.println("Cycle: Scan response received! Proceeding to data request...");
        wakeScanState = CYCLE_DATA_REQUEST;
        cycle_state_time = millis();
      } else if (millis() - cycle_state_time >= RESPONSE_LISTEN_WINDOW) { // 2 second timeout for response
        Serial.println("Scan timeout");
        wakeScanState = CYCLE_SLEEPING;
        cycle_state_time = millis();
      }
      break;
      
    case CYCLE_DATA_REQUEST:
      // Step 4: Send data request after successful scan
      Serial.println("Data request");
      sendDataRequest();
      dataResponseReceived = false;  // Reset flag
      wakeScanState = CYCLE_DATA_LISTENING;
      cycle_state_time = millis();
      break;
      
    case CYCLE_DATA_LISTENING:
      // Step 5: Listen for data response (with timeout)
      if (dataResponseReceived) {
        //Serial.println("Cycle: Data response received! Going to sleep...");
        wakeScanState = CYCLE_SLEEPING;
        cycle_state_time = millis();
      } else if (millis() - cycle_state_time >= RESPONSE_LISTEN_WINDOW) { // timeout for response
        Serial.println("Data timeout");
        wakeScanState = CYCLE_SLEEPING;
        cycle_state_time = millis();
      }
      break;
      
    case CYCLE_SLEEPING:
      // Step 6: Sleep for power conservation with randomized timing to avoid contention
      static unsigned long randomized_sleep_time = 0;
      
      if (!sleepStateEntered) {
        sleepStateEntered = true;

        // Just entered sleep state - generate randomized sleep time
        Serial.println("Sleeping");
        int16_t sleepState = radio.sleep();  // Put radio to sleep to save power
        if (sleepState != RADIOLIB_ERR_NONE) {
          error_message("Enter sleep mode failed", sleepState);
        }
        radioSleeping = true;
        downlinkRxActive = false;
        
        // Add ±25% variation to SLEEP_TIME to avoid synchronized wake-ups
        // Range: 75% to 125% of SLEEP_TIME
        int variation = (SLEEP_TIME / 4);  // 25% of SLEEP_TIME
        randomized_sleep_time = SLEEP_TIME - variation + (rand() % (2 * variation));
        
        // Serial.print("Cycle: Randomized sleep duration: ");
        // Serial.print(randomized_sleep_time);
        // Serial.println("ms");
      }
      
      // Sleep for randomized time, then restart cycle
      if (millis() - cycle_state_time >= randomized_sleep_time) {
        sleepStateEntered = false;
        wakeScanState = CYCLE_WAKING;
        cycle_state_time = millis();
      }
      break;
  }
}
