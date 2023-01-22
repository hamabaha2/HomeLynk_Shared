/*
* ESP32 Sensor Send and AP Pairing.
*
* Device operates in TWO modes:
* 1..PAIR_MODE:
*   Start an AP with SSID & PASS creds and allow other devices to register.
*   Save MAC addresses of newly registering clients in EEPRM MAC array if not there.
*   
* 2..RUN_MODE:
*   Normal operating mode. Read state of sensor and send ESP-NOW message to each paired device.
*   Every main loop delay 3 seconds.
*   
* RUN_MODE is the normal state of the device.  
* Short button press enters "ADD A DEVICE pair mode" BLU LED continuous flash.
* Long button press enters "CLEAR MEMORY" option. ALL LEDs FLASH - paired device array cleared.
* NOTE: HUB must be in pairing mode BEFORE relay device is set to pair!!!
* On pairing success both HUB will exit pairing mode and RELAY boards should restart.
* So pairing of multiple devices must be done one at a time.
* A maximum of 8 devices may be paired.
*   
* PROTOTYPE DEVICES:
*   The HUB just needs an android phone type micro USB power lead to run.
*   The same applies to the ESP8266-MOD2F "RELAY" device.
*   The ESP-01/ESP01S based relay modules need 5 DC on the power terminals. 6 V DC is OK but no more!
*     Relay is an srd-05vdc-sl-c up 10 A switching capability.
*     See https://www.circuitbasics.com/wp-content/uploads/2015/11/SRD-05VDC-SL-C-Datasheet.pdf
*   The Sonoff "Basic" is powered by the mains input which it switches.
*   
*   
* ESP32 LED:
*   BLU:
*     OFF:  Powered Down
*     ON for short double blink mostly OFF: Mains power is OFF
*     OFF for short double blink mostly ON: Mains power is ON
*     Continuous fast blink: PAIRING.
*     
*   GRN:
*     BLINK: Number of quick blinks indicates how many paired devices ARE connected.
*     
*   RED: 
*     BLINK: Number of quick blinks indicates how many paired devices NOT connected.
*   
*   
* ESP-01 LED usage:
*   OFF:  Powered Down
*   ON for short double blink mostly OFF: Connected and mains power is OFF
*   OFF for short double blink mostly ON: Connected and mains power is ON
*   Continuous fast blink: PAIRING.
*   Continuous slow blink: NOT CONNECTED.
* 
* OPERATIUON:
* 
* HUB:
*   Install the HUB attached to the mains cable using the supplied velcro straps.
*   Wrap the sersor cable 6 time round the mains cable.
*   NOTE: The HUB is expected to REMAIN POWERED CONTINUOUSLY while mains power is on or off!!!
*   So it's power supply must NOT be the mains being monitored!
*   
* RELAYS:
*   Power the relay devices and pair with the HUB as needed. 
*   Then position and connect relay to devices to be controlled.
*   Relay is ENERGISED while mains power is ON and de-enegrgised when maisn power is off.
*   Removal of power from the relay device will obviuosly de-energise the relay.
*   
* TO DO:  
*   1,,Do NOT print all 4095s if min/max is 4095/4095! HOW?!!!
*   2..Print total/success/fail delivery totals for EACH MAC that is paired!
*     In the last 100 messages for each paired device count success & fail deliveries.
*     Keep result for previous 100 and running total this 100.
*   ####### TO DO ###### 
*   
* 5.0 WIFI/SERIAL DEBUG FLAGS:
*   Control debug serial output gathered to a buffer.
*   If the WIFI_DEBUG flag is set this debug string is sent via the ESP-NOW message.
*   If SERIAL_DEBUG flag is set print using the Serial module.
*
* Serial / WiFi DEBUG: OUTPUT OPTIOIN:
*   As access to the HUB device may be restricted TWO means of outputing DEBUG text to a paired device is available.
* 1..Append text to the espChars field of the myData ESP-NOW message struct. 
*   This is currently used to send analoge value samples.
*   Eg: sprintf( (myData.espChars + (i * 5)), "%04i ", sensor_analog);
*   This is sent to ALL paired devices in the mains status report and printed to their Serial port.
* 2..Append text to espChars field of myDebug struct using DEBUG__PRINT(APPEND) call
*   This is sent to PAIRED DEVICE #1 ONLY and ONLY on a call of DEBUG__PRINT(SEND);

*   Flag options to send debug print text output to Serail AND/OR #1 Relay device.
*   NOTE: debug / info text can also be gathered in the esp_message struct espChars field sent to ALL Relay devices.
*   
*   
*   %%%%%% Redoing OnDataSent %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <EEPROM.h>

//%%%%%%%%%%%%%% AS of Thu 21st Dec THIS IS CURRENT WORKING VERSION!!! %%%%%%%%%%%%%%%%

#define PROG_ID "ESP32-NOW_HUB_R2_V5.3D"

// 5 New PCB. LED blink pattern changed. 8 devices. XX: No print! A: Pull-up L:LEDs Rev T: Thrsh
// 5.0/1: Back to ground ref thresholds. Serial/WiFi debug output. New msg struct. A: Fix espChars. B: IO9 ADC  C: 3+ < 3500  D: No IO19,20,9 
//  E: Stats to struct. F: Thtreshold 300 < = Mains Off > = Mains ON  G:Thresh 500  H: Tidy comments. WIFI_DEBUG  J: WIFI_DEBUG.
//  K: More PRINT__DEBUG!
// 5.21: SERIALPRINT flag to prevent USB hang!  A: Control with EEPROM flag on reset.  B: Delay on WiF DEBUG send.
// 5.22: Add wait for call-back ACK/NAK. W: WiFi DEBUG False M: MAc ADDR  C: Clear MAC array.  E: EEPROM offset fixed.
// 5.3: Tidied version for release.
// A: Add text buffer over-run check.  B: reVal!  B: SENSOR_LOW 750. sensor_out = true if low!!!  D: 750 -> 1000 on ***ADJUST***

#define TICK_DELAY 100            // Duration of global loop delay. 100 mS = 1 "tick".  C: Find max max count! 30: Samples. TA: New target analysis.
#define MAC_SIZE_IN_BYTES 6                // A MAC address is 6 bytes long!
#define PAIRED_DEVICE_LIMIT 8     // How many paired devices supported.
#define MAC_ARRAY_SIZE (PAIRED_DEVICE_LIMIT * MAC_SIZE_IN_BYTES)  // Allow for up to all MAC addresses in array.
#define EEPROM_SIZE (MAC_ARRAY_SIZE + 2)          // Allow for MAC array, NUMPAIRED cold start flag and SERIAL_MODE_FLAG.
#define EEPROM_SERIAL_FLAG_ADDR 0 // Allow setting of flag if reset btn pressed during boot process.
#define EEPROM_NUMPAIRED_ADDR 1   // Default value of EEPROM byte is 0xFF. Set location to num paired 0-8.
#define EEPROM_MAC_ARRAY_ADDR 2   // Start address of MAC array.
#define SERIAL_MODE_FLAG 0x55     // Pairing mode flag. TRUE if set to this value else FALSE.
#define SERIAL_MODE_DELAY 3000    // Wait this long for user to reset for SERIAL_MODE mode.

#define LED_ON   1                // Allow for LED drive reversal. 0: Mk1 PCB. 1 for MK2 PCB
#define LED_OFF  0                // Ditto
#define APPEND false              // Control DEBUG__PRINT - append text to buffer
#define SEND true                 // Append text to buffer and SEND to Serial and / or WiFi.

// WiFi AP mode params...
#define AP_SSID "ESP32"
#define AP_PASS "magicword"
#define CHANNEL 1
#define IS_HIDDEN false

//#define IS_BREADBOARD     // Allow changes for breadboard version. Node MCU !!!!
#ifdef IS_BREADBOARD
  #define SENSOR_IN 32
  #define LED_RED 21    // Thes pins are NOT RESERVED / INTERNAL on Dev Module.
  #define LED_GRN 22    //  
  #define LED_BLU 23    //  
#else
  #define SENSOR_IN 10  // NOTE: Some Dev modules will NOT WORK with these non-breadboard pins!
  #define LED_RED 13    // 
  #define LED_GRN 12    //
  #define LED_BLU 11    //
#endif

#define LED_ALL 111     // Used by MultiBlink to select ALL three LEDS      
#define BTN_PIN 0       // Pin to select pairing mode control. Short pair, long clear.
#define GPIO19 19       // D+
#define GPIO20 20       // D-
#define GPIO9 9         // Control ADC
#define DEBOUNCE_LIMIT 5 // Debounce delay for button interrupt logic. 5 * TICK_DELAY.
#define BTN_DOWN 0      // While btn pressed GPIO input will read 0.

#define BLINK_DELAY_FAST 250
#define BLINK_DELAY_SLOW 750

#define PAIRING_TIMEOUT_SECS 30   // Limit pairing mode time.
#define LOOP_EXEC_DELAY_SECS 3    // Execute main loop code every 3 seconds

#define SENSOR_LOW 1000             // ***ADJUST*** Analogue value LESS than this when strong induced signal present.
#define SENSOR_MAX 4095             // Analogue value when input at max.
#define SENSOR_TOP_TARGET 5         // Must hit this value to count a top of cycle.
#define SENSOR_TARGET_FREQUENCY 3   // Three cycles at 50 Hz in 60 mS sample timne.
#define SENSOR_MAX_CNT_THRESHOLD 10 // Less than this count => MAINS ON else OFF!
#define MAX_LOW_COUNT_TARGET 7      // If count of consecutive readings = MAX < this MAIN IS ON!
#define LOW_COUNT_TARGET 2          // If count of consecutive readings < SENSOR_LOW > this MAINS IS ON!
#define MAX_MAX_COUNT_TARGET 12     // If count of consecutive readings = MAX < this MAIN IS ON!


#define MSG_COUNT_LIMIT 100     // Count up to this limit and analyes totals.
#define MSG_TIMEOUT_MS 50       // Wait this many mS for confirmed msg delivery.

#define DEBUG_DEVICE_INDEX 0    // This paired device is used for debug data messages.
#define DEBUG_MSG 0             // This message contains debug data to be printed.
#define MAINS_DATA_MSG 1        // This message contains mains status data to be used for device output control.
#define MSG_DELIVERY_PENDING 0    // Set by msg send calling code. Updated by callback fn.
#define MSG_DELIVERY_SUCCESS 1    // Got ACK from target device.
#define MSG_DELIVERY_FAILURE 2    // No ACK from target device.
#define TEXT_BUFFER_SIZE 200    // Use text buffers of this size AND CHK FOR OVER-RUN

bool SERIALPRINT = false;       // Prevents serial output on USB port blocking program execution. True on EEPROM reset!
bool WIFI_DEBUG = true;         // Do NOT use the ESP-NOW msg to print debug ouitput if false.
bool SERIAL_DEBUG = true;       // Do NOT use the Serial module to print debug ouitput if false.
int loopCount = 0;              // Main loop counter.
bool btnPressed = false;        // Flag set by interrupt code on button pressed.
int lastMsgStatus = 0;          // Set by calling & callback fn. 0 = PENDING, 1 = GOOD, 2 = BAD.

int sensor_analog = 0;      // Value of analogue read.
int sensor_max = 4095;      // Max value read during one analysis.
int sensor_min = 0;         // Ditto min.
bool sensor_out = false;    // Value output by the sensor for mains detected true or not false.
int sensorCycleCount = 0;   // Count cycles during one analysis period.
int sensorIsMaxCount = 0;   // While sensor read is at max. Count when this happens.
int sensorIsTopCount = 0;   // Count how many times above counter = SENSOR_TOP_TARGET
int sensorIsLTTCount = 0;   // Is Less Than Threshold count.
int sensorIsMaxCountMax = 0;
int sensorIsLowCount = 0;     // Count while low.
int sensorIsLowCountMax = 0;  // Highest count while low.

// Sample working ESP-NOW target MAC address.
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xAB, 0x98, 0x64}; // 40:91:51:AB:98:64


// Add paired MAC addresses to this array 6 bytes per address.
// Read from EEPROM on boot. Write/update to EEPROM on new pair.
int pairedItemCount = 0;                            // NOTE: MAC_ARRAY_SIZE defines max number of MAC addreses * MAC size(6).
uint8_t pairedMACAddressArray[MAC_ARRAY_SIZE] = {}; // Save paired MAC addreses in RAM. Copy to EEPROM when changed.

// STATISTICS::
int msgDeviceIndex = 0;                       // Which switch device is currently being targeted.
int msgCount = 0;                             // Count messages up to 100. COMMON count - each switch gets a copy!
int msgSuccess[PAIRED_DEVICE_LIMIT] = {};     // Running total success delivery.
int msgFail[PAIRED_DEVICE_LIMIT] = {};        // Ditto fail delivery.
int last100Success[PAIRED_DEVICE_LIMIT] = {}; // Results of last 100 msg deliveries success.
int last100Fail[PAIRED_DEVICE_LIMIT] = {};    // Ditto fail.

char buffer[TEXT_BUFFER_SIZE];                 // Working text buffer for debug print - DO NOT OVERFLOW!.

// Structure to send data.
// Must match the receiver structure.
typedef struct esp_message {
  byte espType;           // MAINS_DATA_MSG => DEBUG. 1 => Mains data.
  bool espMains;          // If type == 1 this is mains state false = OFF, true = ON.
  char espChars[TEXT_BUFFER_SIZE];     // Text message DEBUG serial out.
};

// Create a struct_message called myData
esp_message myData;   // Send mains status to remote devices.
esp_message myDebug;  // Send debug / info data to remote device #1.

// Create global variables to hold WiFi data...
esp_now_peer_info_t       peerInfo;
wifi_sta_list_t           wifi_sta_list;
tcpip_adapter_sta_list_t  adapter_sta_list;


// Send struct ESP-NOW message to specifiec paired device.
bool sendESPMessage(int pairedDevNo, esp_message myMsg) {
  bool retVal = false;  // Set true if send is good.
  // Copy MAC address to working target MAC var and print.
  for(int j = 0; j < MAC_SIZE_IN_BYTES; j++) {
    broadcastAddress[j] = pairedMACAddressArray[ (pairedDevNo * MAC_SIZE_IN_BYTES) + j];
    if (SERIAL_DEBUG) {
      if (SERIALPRINT) Serial.printf("%02X",  pairedMACAddressArray[ (pairedDevNo * MAC_SIZE_IN_BYTES) + j]);  
      if ( j < MAC_SIZE_IN_BYTES - 1 ) {
        if (SERIALPRINT) Serial.print(":");
      } else {
        if (SERIALPRINT) Serial.println();      
      }
    }
  }
  // Copy to peerInfo - IS THIS NEEDED?
  memcpy(peerInfo.peer_addr, broadcastAddress, MAC_SIZE_IN_BYTES);
  lastMsgStatus = MSG_DELIVERY_PENDING;    // Updated by callback fn.
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myMsg, sizeof(myMsg));
  if (result == ESP_OK) {
    if (SERIALPRINT) Serial.print("Send result: SUCCESS... ");
    int CB_WaitLoop = 0;
    while ( (lastMsgStatus == MSG_DELIVERY_PENDING) && (CB_WaitLoop < MSG_TIMEOUT_MS) ) {
      delay(1);
      CB_WaitLoop++;
    }
    if (SERIALPRINT) Serial.printf("ESP-NOW response time: %i\n", CB_WaitLoop);    
    if (SERIALPRINT) Serial.print(" Delivery: ");
    if (lastMsgStatus == MSG_DELIVERY_SUCCESS) {
      if (SERIALPRINT) Serial.println("GOOD");
    } else {
      if (SERIALPRINT) Serial.println("BAD");            
    }
    retVal = true;
  } else {
     if (SERIALPRINT) Serial.print("Send result: FALED!!!");   
  }
  return retVal;
}


// Debug text is printed to the myDebug.espChars buffer. Eg: sprintf(myDebug.espChars, "%s\n", "Setup exiting...");
// Then call to this fn. Flags control Serial and ESP-NOW message output.
// If sendFlag true text is printed to Serial and or sent to paired device #0
bool DEBUG__PRINT(bool sendFlag) {
  bool retVal = true;
  if (strlen(myDebug.espChars) + strlen(buffer) > TEXT_BUFFER_SIZE + 1) {
    retVal = false;
  } else {
    strcat(myDebug.espChars, buffer);
    if (SERIAL_DEBUG && sendFlag) {
      if (SERIALPRINT) {
        Serial.print(myDebug.espChars);
        Serial.flush();
      }
    }
    if (WIFI_DEBUG && sendFlag) {
      myDebug.espType = DEBUG_MSG; 
      sendESPMessage(DEBUG_DEVICE_INDEX, myDebug);
      delay(500); // Need delay for delivery of message before buffer overwritten!!!    ????
    }
    if (sendFlag) {
      buffer[0] = 0;            // Text string terminator. Make buffer empty string.
      myDebug.espChars[0] = 0;  // Ditto.
    }
  }
  return retVal;
}



// Read EEPROM paired MAC array into RAM.
void readEEPROMArray() {
  pairedItemCount = EEPROM.read(EEPROM_NUMPAIRED_ADDR);         // Location 1 is number of MAC paired.
  if ( pairedItemCount > (MAC_ARRAY_SIZE / MAC_SIZE_IN_BYTES) ) {         
    if (SERIALPRINT) Serial.printf("ILLEGAL pairedItemCount in EEPROM: %i - SET to ZERO!!", pairedItemCount);
    pairedItemCount = 0;                                        // Abort if array size is illegal.
  }
  for (int i = 0; i < (pairedItemCount * MAC_SIZE_IN_BYTES); i++) {
    pairedMACAddressArray[i] = EEPROM.read(EEPROM_MAC_ARRAY_ADDR + i);                // Copy array from EEPROM to RAM.
  }
}



// Update EEPROM MAC array from RAM on change
void updateEEPROMArray() {
  EEPROM.write(EEPROM_NUMPAIRED_ADDR, pairedItemCount);
  for (int i = 0; i < MAC_ARRAY_SIZE; i++) {
    EEPROM.write(EEPROM_MAC_ARRAY_ADDR + i, pairedMACAddressArray[i]);    // Update value in EEPROM - only writes if changed.
  }
  EEPROM.commit(); 
}


// Clear/copy the msg analysis arrays...If true clear teh last 100 else copy new totals.
void clearMsgAnalysis(bool last100) {
  // End of 100 msg analysis / new pairing - copy results to last result arrays & clr totals...
  for (int x = 0; x < PAIRED_DEVICE_LIMIT; x++) {
    if (last100) {
      last100Success[x] = 0;
      last100Fail[x] = 0;      
    } else {
      last100Success[x] = msgSuccess[x];
      last100Fail[x] = msgFail[x];
    }
    msgSuccess[x] = 0;
    msgFail[x] = 0;
  }
  msgCount = 0;
}


// MULTI-BLINK: Blink one or all LED with arg for delay. Start state is ON.
// Specify LED pin - 0 => ALL
void multiBlinkLED (int LEDPinNo, int delayMS, int blinkCount) {
    // Save current states...
    int LED_RED_WAS = digitalRead(LED_RED);
    int LED_BLU_WAS = digitalRead(LED_BLU);
    int LED_GRN_WAS = digitalRead(LED_GRN);
    // Loop blinkCount times with one or all LEDs toggled.
    for (int i = 0; i < blinkCount; i++) {
      if (LEDPinNo == LED_ALL) {
        digitalWrite(LED_RED, LED_ON);
        digitalWrite(LED_BLU, LED_ON);
        digitalWrite(LED_GRN, LED_ON);
      } else {
        digitalWrite(LEDPinNo, LED_ON);
      }
      delay(delayMS);
      if (LEDPinNo == LED_ALL) {
        digitalWrite(LED_RED, LED_OFF);
        digitalWrite(LED_BLU, LED_OFF);
        digitalWrite(LED_GRN, LED_OFF);
      } else {
        digitalWrite(LEDPinNo, LED_OFF);
      }
      delay(delayMS);      
    }
    // Restore LEDs to original state...
    digitalWrite(LED_RED, LED_RED_WAS);
    digitalWrite(LED_BLU, LED_BLU_WAS);
    digitalWrite(LED_GRN, LED_GRN_WAS);
}


// Compare MAC address for this station to all sets in pairedMACAddressArray.
bool compareMac(tcpip_adapter_sta_info_t station) {
  // For each MAC 6 byte set in array...
  bool newMAC = true;     // Assume this is a new MAC unless we find a match in the array.
  int byteMatchCount = 0;
  // For each paired MAC address in array... 
  for (int ndxPairedMAC = 0; ndxPairedMAC < pairedItemCount * MAC_SIZE_IN_BYTES; ndxPairedMAC += MAC_SIZE_IN_BYTES) {
    // For each byte of 6 byte MAC...
    for(int j = 0; j < MAC_SIZE_IN_BYTES; j++) {
      if (station.mac[j] == pairedMACAddressArray[ndxPairedMAC + j]){
        byteMatchCount++;
      }
    }
    if (byteMatchCount == MAC_SIZE_IN_BYTES) {
      // We have a match!
      newMAC = false;
      if (SERIALPRINT) Serial.printf("Found a match!\n");
      break;     // Exit OUTER FOR LOOP
    }
    byteMatchCount = 0;
  }
  return newMAC;
}


// Compare new pair request MAC to those in array. 
// Add MAC address to end of array if no match found and update EEPROM copy.
// Use global adapter_sta_list CHECK space available.
bool pairNewMac() {
  bool newMAC = false;
  // For each station (device registered with AP)...
  for (int ndxSTA = 0; ndxSTA < adapter_sta_list.num; ndxSTA++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[ndxSTA];
    newMAC = compareMac(station);
    if (newMAC) {
      // This MAC is NOT already paired so print it out
      if (SERIALPRINT) Serial.print("NEW MAC: ");
      for(int ndxMAC = 0; ndxMAC < MAC_SIZE_IN_BYTES; ndxMAC++) {
        if (SERIALPRINT) Serial.printf("%02X", station.mac[ndxMAC]);  
        if (ndxMAC < (MAC_SIZE_IN_BYTES - 1) ) {
          if (SERIALPRINT) Serial.print(":");
        }
      }
      if (SERIALPRINT) Serial.println();
      if (pairedItemCount == PAIRED_DEVICE_LIMIT) {
        if (SERIALPRINT) Serial.println("PAIRED ARRAY FULL!!!");        
      } else {
        // Add new MAC address to paired MAC array...
        for(int ndxMAC = 0; ndxMAC < MAC_SIZE_IN_BYTES; ndxMAC++) {
          pairedMACAddressArray[pairedItemCount*MAC_SIZE_IN_BYTES + ndxMAC] = station.mac[ndxMAC];
          broadcastAddress[ndxMAC] =  station.mac[ndxMAC];        
        }
        pairedItemCount++;
        updateEEPROMArray();
        if (SERIALPRINT) Serial.printf("Size of array = %i\n", pairedItemCount );
        if (SERIALPRINT) Serial.println("EXIT PAIRING...clearing message delivery analyis.");        
        clearMsgAnalysis(false);
        digitalWrite(LED_BLU, LED_ON);      
        delay(3000);  // Wait before restarting... arbitrary.
        WiFi.mode(WIFI_STA);
        break;
      }
    }
  }
  return newMAC;    
}

// callback when data is sent
// %%%% byte result 0 = pending, 1 = success, 2 = failed, 3 = timed-out. %%%%
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
  if (sendStatus == ESP_NOW_SEND_SUCCESS) {
    lastMsgStatus = MSG_DELIVERY_SUCCESS;
    msgSuccess[msgDeviceIndex]++;
  }
  else {
     lastMsgStatus = MSG_DELIVERY_FAILURE;
     msgFail[msgDeviceIndex]++;
  }
} 
  

// When button predded. Wait until released and react based on LONG or SHOR press.
void buttonInt() {
  btnPressed = true;
}

void btnDown() {
  int LED_RED_WAS = digitalRead(LED_RED);
  int LED_BLU_WAS = digitalRead(LED_BLU);
  int LED_GRN_WAS = digitalRead(LED_GRN);
  
  int count = 0;
  digitalWrite(LED_RED, LED_ON);
  digitalWrite(LED_BLU, LED_ON);
  digitalWrite(LED_GRN, LED_ON);
//==== Button Down Logic =================================================================================
  while ( (digitalRead(BTN_PIN) == BTN_DOWN) || (count < DEBOUNCE_LIMIT) ) {
    count++;
    delay(TICK_DELAY);
    if (count > 20) {
      digitalWrite(LED_RED, LED_OFF);
    }
    if (count > 30) {
      digitalWrite(LED_BLU, LED_OFF);
    }
    if (count > 40) {
      digitalWrite(LED_GRN, LED_OFF);
    }
  }
//--------------------------------------------------------------------------------------------------------
  if (SERIALPRINT) Serial.printf("count = %i", count);

// ==== PAIRING LOGIC =======================================================================================
  if (count < 20) {
    // Quick press < 2 second PAIRING MODE"! Loop at 10 Hz flashing BLU LED and wait for WiFi register on AP
    // Loop until new MAC address found or timeout.
    if (SERIALPRINT) Serial.print("Pairing mode...");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, CHANNEL, IS_HIDDEN);
    
    
    // Check if space available in MAC array.
    if (pairedItemCount == PAIRED_DEVICE_LIMIT) {
      if (SERIALPRINT) Serial.print("MAC ARRAY FULL!!!");
    } else {
      multiBlinkLED(LED_BLU, BLINK_DELAY_FAST, 3);
      bool newMACAddress = false;
      count = 0;  // Loop count for timeout
      while ( (count < (PAIRING_TIMEOUT_SECS * 1000 / TICK_DELAY)) && (newMACAddress == false) ) {
        count++;
        // Clear AP working vars.
        memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
        memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
        // Copy AP data for working use...
        esp_wifi_ap_get_sta_list(&wifi_sta_list);
        tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
        // Check for a new station device registered with AP...
        for (int i = 0; i < adapter_sta_list.num; i++) {
          tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
          // Print index/num and MAC address of new device found...
          if (SERIALPRINT) Serial.printf("station nr %i  ",i);
          if (SERIALPRINT) Serial.print("MAC: ");
          for(int i = 0; i < MAC_SIZE_IN_BYTES; i++){
            if (SERIALPRINT) Serial.printf("%02X", station.mac[i]);  
            if ( i<5 && SERIALPRINT) Serial.print(":");
          }
          if (SERIALPRINT) Serial.println();
          newMACAddress = pairNewMac(); // Check if this is a new 
        }
        digitalWrite(LED_BLU, !digitalRead(LED_BLU));   // Rapid blink LED while waiting...
        delay(TICK_DELAY);
      }
    }
    initWiFi();   // Back to ESP-NOW mode...    
  } else {
    // Slow press and hold > 4 second - FLASH ALL LEDS and clear MAC table.
    if (count > 40) {
      multiBlinkLED(LED_ALL, BLINK_DELAY_FAST, 3);
//==== CLEARING LOGIC ===============================================================================          
      // Held long enough - CLEAR MAC address table the exit.
      if (SERIALPRINT) Serial.print("Clear pairing data...");
      // DO MAC CLEAR - Zero addresses!
      for (int i = 0; i < MAC_ARRAY_SIZE; i++) {
        pairedMACAddressArray[i] = 0;
      }
      pairedItemCount = 0;
      updateEEPROMArray();
      // Clear ESP-NOW settings...
      esp_now_deinit();
      // Init ESP-NOW
      if (esp_now_init() != ESP_OK) {
        if (SERIALPRINT) Serial.println("Error initializing ESP-NOW");
        return;
      }
      // Once ESPNow is successfully Init, we will register for Send CB to
      // get the status of Trasnmitted packet
      esp_now_register_send_cb(OnDataSent);    
    }
//----------------------------------------------------------------------------------------------------    
    else {
      if (SERIALPRINT) Serial.print("Not long enough for clearing - abort!");    // Btn press  NOT long enough for clearing request!
    }
  }
  btnPressed = false; // Interrupt handled so clear flag and restore LEDs to original pstate.
  digitalWrite(LED_RED, LED_RED_WAS);
  digitalWrite(LED_BLU, LED_BLU_WAS);
  digitalWrite(LED_GRN, LED_GRN_WAS);
}


void initWiFi() {
 // Set up WiFi - MAY WANT TO Do AP ONLY WHEN PAIRING!!!???
  WiFi.mode(WIFI_STA);

  // Ensure clean start and empty peer collection
  if (SERIALPRINT) Serial.printf("de-init ESP-NOW: %d\n",   (esp_now_deinit() == ESP_OK) );
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    if (SERIALPRINT) Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Ensure peer array is empty...
  while (esp_now_is_peer_exist(0) ) {
    esp_now_del_peer(0);
  }


  // List existing paired device MAC addresses...
  for (int i = 0; i < pairedItemCount; i++) {
    if (SERIALPRINT) Serial.printf("MAC address for paired device # %i ", i + 1);
    for(int j = 0; j < MAC_SIZE_IN_BYTES; j++) {
      broadcastAddress[j] = pairedMACAddressArray[ (i * MAC_SIZE_IN_BYTES) + j];
      if (SERIALPRINT) Serial.printf("%02X", pairedMACAddressArray[ (i * MAC_SIZE_IN_BYTES) + j] );  
      if (j<5 && SERIALPRINT) Serial.print(":");
    }
    if (SERIALPRINT) Serial.println();
    // Register peer - THIS IS NEEDED!!!
    memcpy(peerInfo.peer_addr, broadcastAddress, MAC_SIZE_IN_BYTES);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer - THIS IS NEEDED!!!       
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      if (SERIALPRINT) Serial.println("Failed to add peer");
      return;
    }
  }
  
}


void setup() {
  pinMode(SENSOR_IN, INPUT_PULLUP);
  pinMode(GPIO19, INPUT_PULLUP);
  pinMode(GPIO20, INPUT_PULLUP);
//  pinMode(GPIO9, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  digitalWrite(LED_RED, LED_OFF);  // Set initial state
  digitalWrite(LED_GRN, LED_OFF);
  digitalWrite(LED_BLU, LED_OFF);

  multiBlinkLED(LED_ALL, BLINK_DELAY_FAST, 2); // Show sign of life!

  EEPROM.begin(EEPROM_SIZE);
  // Check EEPROM to see if this is a double reset SERIALPRINT flag event...
  if (EEPROM.read(EEPROM_SERIAL_FLAG_ADDR) == SERIAL_MODE_FLAG) {
    // Turn off pairing mode flag so reset will abort - otherwise we get back here!
    EEPROM.write(EEPROM_SERIAL_FLAG_ADDR, 0xFF);
    EEPROM.commit();
    SERIALPRINT = true;     // Set flag to see serial output - MUST BE USB TO PC/LAPTOP or HANG!!!!
  } else {
    // Set the SERIAL_MODE_FLAG...
    EEPROM.write(EEPROM_SERIAL_FLAG_ADDR, SERIAL_MODE_FLAG);
    EEPROM.commit();
  }
  digitalWrite(LED_BLU, LED_ON);
  delay(SERIAL_MODE_DELAY);  // !!!!! IF RESET NOW WILL INVOKE SERIAL_MODE!!!!!
  digitalWrite(LED_BLU, LED_OFF);
  // Recind option to SERIAL_MODE!!!
  EEPROM.write(EEPROM_SERIAL_FLAG_ADDR, 0xFF);
  EEPROM.commit();

  
  // Init Serial Monitor
  if (SERIALPRINT) Serial.begin(115200);
  if (SERIALPRINT) while (!Serial) {;}
  if (SERIALPRINT) Serial.printf("EEPROM_NUMPAIRED_ADDR = %i\n", EEPROM.read(EEPROM_NUMPAIRED_ADDR)); // Show stored array size.

  if (EEPROM.read(EEPROM_NUMPAIRED_ADDR) != 0xFF) {
    // This is NOT a cold start so read MAC address arary from EEPROM to RAM...
    readEEPROMArray();  // Includes pairedItemCount.
  }


  
  
  initWiFi();
  delay(100);   // Arbitrary wifi ready delay ????

  clearMsgAnalysis(true);
  // Set up button interrupt on GPIO pin.
  attachInterrupt(BTN_PIN, buttonInt, FALLING);  // Pin has pull-up res. LOW when pressed.

  sprintf(buffer, "\n\n%s\n", PROG_ID);
  DEBUG__PRINT(APPEND);
  sprintf(buffer, "EEPROM_SERIAL_FLAG_ADDR = %i\n", EEPROM.read(EEPROM_SERIAL_FLAG_ADDR));
  DEBUG__PRINT(SEND);

  sprintf(buffer, "%s\n", " ----> Setup exiting...");    // MISSING ON REMOTE DEVICE... W H Y ? ? 
  DEBUG__PRINT(SEND);
}

// Handle sensor state input, detector analysis and ESP-NOW message transmission...
void mainLoopCode() {
  if (SERIALPRINT) Serial.println();
  if (SERIALPRINT) Serial.println();
  sensor_out = false;       // State of mains detector - default "OFF".
  sensor_max = 0;       // Max value of analogue input during analysis cycle.
  sensor_min = 4095;    // Ditto Min value.
  sensorCycleCount = 0; // Count number of cycles during analysis period.
  sensorIsMaxCount = 0; // While sensor read is at max. Count when this happens.
  sensorIsMaxCountMax = 0; // Find highest count for sequence.
  sensorIsLowCount = 0; // Less than threshold.
  sensorIsLowCountMax = 0; // Highest count while low.
  // Loop for 60 mS at 1 mS intervals to analyse analogue input from meains sensor.
  int i;  // USE OUTSIDE VLOOP
  for (i = 0; i < 60; i++) {
    sensor_analog = analogRead(SENSOR_IN);
    
    if (SERIALPRINT) Serial.print(sensor_analog);
    Serial.print ("  ");
    // Measure length of value at MAX...
    if (sensor_analog == SENSOR_MAX) {
      sensorIsMaxCount++;
      if (sensorIsMaxCount > sensorIsMaxCountMax) {
        sensorIsMaxCountMax = sensorIsMaxCount;
      }
    } else {
      sensorIsMaxCount = 0;
    }
    // Measure length of value at LOW...
    if (sensor_analog < SENSOR_LOW) {
      sensorIsLowCount++;
      if (sensorIsLowCount > sensorIsLowCountMax) {
        sensorIsLowCountMax = sensorIsLowCount;
      }
    } else {
      sensorIsLowCount = 0;
    }
    // If at max value long enough count as one mains cycle.
    if (sensorIsMaxCount == SENSOR_TOP_TARGET) {
      sensorCycleCount++;
    }
    // Find max value in this period...
    if (sensor_analog > sensor_max) {
      sensor_max = sensor_analog;
    }
    // Find min value in this period...
    if (sensor_analog < sensor_min) {
      sensor_min = sensor_analog;
    }
    delay(1);
    // #### Add first 30 values to msgBuf ####
    if (i < 30) {
      sprintf( (myData.espChars + (i * 5)), "%04i ", sensor_analog);
    }
  }
  sprintf( (myData.espChars + (30 * 5)), "Min: %04i  MaxCnt: %04i", sensor_min, sensorIsMaxCountMax);
  // ANALYSE the signal detected send results accordingly.
  if ( (sensorIsLowCountMax > LOW_COUNT_TARGET) ) {
    sensor_out = true;   // If we have found a multiple values re. threshold set mains flag accordingly..    
  } else {
    sensor_out = false;
  }
  
  if (SERIALPRINT) Serial.println();
  if (SERIALPRINT) Serial.printf("Sensor min = %i  max = %i \n", sensor_min, sensor_max);
  // Set BLUE LED to match detected state of mains.
  if (sensor_out) {
    digitalWrite(LED_BLU, LED_ON);
  } else {
    digitalWrite(LED_BLU, LED_OFF);    
  }
  // Set values to send in ESP-NOW message.
//  strcpy(myData.espChars, "THIS IS AN ESP MSG");
  myData.espType = 1;
  myData.espMains = sensor_out;
  
  // Send message via ESP-NOW to each paired device...
  // Analysis...
  msgCount++;
  if (msgCount > MSG_COUNT_LIMIT) {
    clearMsgAnalysis(false);
    msgCount = 1;
  }
  if (SERIALPRINT) Serial.printf("Sending to %i paired devices...\n", pairedItemCount);
  for (int i = 0; i < pairedItemCount; i++) {
    msgDeviceIndex = i;
    if (SERIALPRINT) Serial.printf("Sending to device # %i ", i + 1);
    sendESPMessage(i, myData);
    // Print msg delivery analysis...
    if (SERIALPRINT) Serial.printf("Total success %i / fail %i\n" , msgSuccess[i], msgFail[i]);
    if (SERIALPRINT) Serial.printf("Last 100 success %i / fail %i\n" , last100Success[i], last100Fail[i]);
    // Show status of msg delivery on LED_GRN - short flash good / long flash bad. 
    // lastMsgStatus set in callback fn.
    if (lastMsgStatus == 1) {
      multiBlinkLED(LED_GRN, TICK_DELAY, 1);
    } else {
      multiBlinkLED(LED_RED, TICK_DELAY, 1);      
    }
    delay(10);  // Arbitrary delay to allow time to settle before next Xmit needed? ????
  }
  // Indicate that the device is operating...
  digitalWrite(LED_BLU, !digitalRead(LED_BLU));   // Rapid blink LED while waiting...
  delay(TICK_DELAY);  
  digitalWrite(LED_BLU, !digitalRead(LED_BLU));   // Rapid blink LED while waiting...
}


// Basic loop: Execute main code at selected frequency eg every 3 seconds...
// Check if button pressed and execute code if flag set.
void loop() {
  loopCount++;
  if (loopCount > (LOOP_EXEC_DELAY_SECS * 1000 / TICK_DELAY) ) {
    // Reset loop counter and execute main loop code after loop delay.
    loopCount = 0;
    mainLoopCode();
  }
  if (btnPressed) {
    btnDown();
  }
  delay(TICK_DELAY);  
}
