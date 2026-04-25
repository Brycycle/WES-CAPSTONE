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




/*~~~~~Radio Configuration~~~~~*/

// Initialize SX1262 radio
// Make a custom SPI device because *of course* Heltec didn't use the default SPI pins
SPIClass spi(FSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0); // Defaults, works fine
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiSettings);




/*~~~~~Function Prototypes~~~~~*/
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

void configureRadioChannel(float freq, float bw, uint8_t sf) {
  // Wake radio from sleep mode if needed before configuration
  int16_t state = radio.standby();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Warning: Failed to wake radio from sleep: %d\n", state);
  } else {
    Serial.println("Radio woken up from sleep mode");
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


  // Start on downlink channel (always listening for activity)
  // start continuous reception
  resumeReception();
  Serial.println("Complete!");
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

      // TODO: handle parsed packet
      (void)receivedPacket;
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

  // Handle button presses to start transmissions
  if(buttonFlag) {
    buttonFlag = false;

    // TODO: add transmit behavior here
  }
}
