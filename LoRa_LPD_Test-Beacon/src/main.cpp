#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include "BeaconHeader.h"


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
bool TXToggle = false; // Toggle with button press to start or stop TX loop.
bool inTXTestLoop = false; // Internal flag to track if in the TX loop (used to control RX logic)
enum ChannelMode { TX_MODE, ACK_MODE };
volatile ChannelMode currentChannel = ACK_MODE; //start in ACK mode



/*~~~~~Radio Configuration~~~~~*/

// Initialize SX1262 radio
// Make a custom SPI device because *of course* Heltec didn't use the default SPI pins
SPIClass spi(FSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0); // Defaults, works fine
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiSettings);




/*~~~~~Function Prototypes~~~~~*/
void configureRadioChannel(float freq, float bw, uint8_t sf);
void error_message(const char* message, int16_t errorCode);
bool switchToTXlinkChannel();
bool switchToACKlinkChannel();
void configureRadioChannel(float freq, float bw, uint8_t sf);


/*~~~~~Interrupt Handlers~~~~~*/
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


////////////*~~~~~Helper Functions~~~~~*/

void error_message(const char* message, int16_t errorCode) {
    Serial.print("Error: ");
    Serial.print(message);
    Serial.print(" (Code: ");
    Serial.print(errorCode);
    Serial.println(")");
}

void configureRadioChannel(float freq, float bw, uint8_t sf) {
  /*
  // Wake radio from sleep mode if needed before configuration
  int16_t state = radio.standby();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Warning: Failed to wake radio from sleep: %d\n", state);
  } else {
    Serial.println("Radio woken up from sleep mode");
  }
  */
  
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

// Transmits predetermined packet and listens for ACK from remote unit. Built in timeout. Returns ACK message (or error message if timeout occurs)
void TXandListenforACK() {
  String ACKmsg = "No ACK\n";

  // Transmit test packet
  switchToTXlinkChannel();
  int16_t state = radio.transmit(TEST_PACKET_10B);
  if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("Transmission failed: %d\n", state);
  }
  else {
      Serial.println("Packet transmitted, waiting for ACK...");
  }

  // Start listening for response
  switchToACKlinkChannel();
  resumeReception();
  unsigned long startTime = millis();
  
  // Wait for response within listen window
  while(millis() - startTime < RESPONSE_LISTEN_WINDOW) {
      if(receivedFlag) {
          receivedFlag = false;
          
          String received_data;
          state = radio.readData(received_data);
          if (state == RADIOLIB_ERR_NONE) {
              // packet was successfully received
              //Serial.println("Received packet!");
              ACKmsg = received_data;
          }
          else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
              // timeout occurred while waiting for a packet
              Serial.println("timeout!");
              ACKmsg = "ACK timeout";
          } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
              // packet was received, but is malformed
              Serial.println("CRC error!");
              ACKmsg = "ACK CRC error";
          } else {
              // some other error occurred
              Serial.print("failed, code ");
              Serial.println(state);
              ACKmsg = "ACK reception failed with error code " + String(state);
          }
          resumeReception();  
      }
      delay(10); // Small delay to prevent busy waiting
  }
  Serial.print(ACKmsg);
  switchToTXlinkChannel();
  resumeReception();
}

void resumeReception() {
  int16_t state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
      error_message("Resuming reception failed", state);
    }
}

void generateandTXACK(String packet_data) {
  String ACKmsge = "ACK BER: ";
  // Check packet integrety
  double ber = calcBER(packet_data);

  // Transmit ACK
  switchToACKlinkChannel();
  String fullACK = ACKmsge + String(ber) + "\n";
  int16_t state = radio.transmit(fullACK.c_str());
  if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("Transmission failed: %d\n", state);
  }
  else {
      Serial.println("ACK transmitted: " + fullACK);
  }
  switchToTXlinkChannel();
  resumeReception();
}

double calcBER(String packet_data) {
  // Placeholder function to calculate BER. 
  return 0.005; // Assume perfect reception for now
}

bool switchToTXlinkChannel() {
  if (currentChannel == TX_MODE) return true;
  
  //Serial.println("Switching to uplink channel...");
  configureRadioChannel(TX_FREQ, TX_BW, TX_SF);
  currentChannel = TX_MODE;
  return true;
}

// Switch to downlink channel (for listening for activity)
bool switchToACKlinkChannel() {
  if (currentChannel == ACK_MODE) return true;
  
  //Serial.println("Switching to downlink channel...");
  configureRadioChannel(ACK_FREQ, ACK_BW, ACK_SF);
  currentChannel = ACK_MODE;
  return true;

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
  Serial.print("Initializing radio with TX channel (ready for TX or listen)...");
  int16_t state = radio.begin(TX_FREQ, TX_BW, TX_SF, 5, 0x34, 1, 8);
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

  Serial.println("Complete! Waiting to ACK. Press button to start TX loop.");
  resumeReception();
}

void loop() {

  //Tx and wait for ACK if toggle is on
  if(TXToggle) {
      TXandListenforACK();
  }

  // Handle reception and send ACK 
  if (!TXToggle && receivedFlag && inTXTestLoop) {
    receivedFlag = false;

    // you can receive data as an Arduino String
    String packet_data;
    int state = radio.readData(packet_data);

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println("Received packet!");
      generateandTXACK(packet_data);
      
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

  // Handle button presses to start/stop Tx test loop
  if(buttonFlag) {
    buttonFlag = false;
    TXToggle = !TXToggle;
    inTXTestLoop = TXToggle; // Update internal flag to match toggle state
    if (TXToggle) {
      Serial.println("Starting transmission loop...");
    } else {
      Serial.println("Stopping transmission loop...");
    }
  }
}
