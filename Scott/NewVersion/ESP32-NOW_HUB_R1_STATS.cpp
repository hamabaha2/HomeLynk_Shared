/*
* ESP32 VSense HUB Controller
* 
*   220VAC Sensor Detector, ESP_NOW Messager Send and AP Pairing.
*
* Device operates in TWO modes:
* 1..PAIR_MODE:
*   Start an AP with SSID & PASS creds and allow other devices to register.
*   Save MAC addresses of newly registering clients in EEPRM MAC array if not there.
*   
* 2..RUN_MODE:
*   Normal operating mode. Read state of sensor and send ESP-NOW message to each paired device.
*   Main loop runs after LOOP_EXEC_DELAY_SECS delay in seconds.
*   
* RUN_MODE is the normal state of the device.
*   Monitors the AC antenna sensor input on the generator LIVE cable. Send state OFF, ON / HOT every loop.
*   Blue LED indicates state of the monitored cable: Mostly ON => cable live is HOT. Mostly OFF => cable live is OFF.
*   
* PAIRING MODE:
*   Short button press in RUN_MODE enters "ADD A DEVICE / PAIR MODE" BLU LED continuous fast flash.
* HOLD button down starts "CLEAR MEMORY" option. ALL LEDs ON then each goes out in sequence. 
* IF button held until last LED turns off then the paired device array is cleared.
* NOTE: HUB must be in pairing mode BEFORE relay device is set to pair!!!
* On pairing success HUB will exit pairing mode and RELAY board will restart.
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
* ====== ESP32 LED BEHAVIOUR ======
* 
* <<< ON BOOT >>>
* All LEDs blink twice.
* GRN on for 3 seconds. 
*   Press btn while LED on to set "Live OFF" reference value.
*   
* BLU on for 3 seconds. 
*   Press btn to set SERIAL DEBUG MODE.
*   NOTE: HUB must be powered by a PC/Laptop to accept serial output. Device WILL HANG if just powered by USB plug!
* 
* After selection process... 
*   GRN LED blinks fast 3 times to confirm set ref value.
*   BLU LED blinks fast 3 times to confirm set DEBUG PRINT.
*   
* <<< RUNNING >>>
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
*   Wrap the sersor cable round the GENERATOR mains cable as many time as possible.
*   Squeeze the loops together to make as neat a coil as possible.
*   
* RELAYS:
*   Power the relay devices and pair with the HUB as needed. 
*   Then position and connect relay to devices to be controlled.
*   Relay is ENERGISED ONLY while mains power is ON and gererator is off.
*   Removal of power from the relay device will obviously de-energise the relay.
*   When power is restored to the relay devices they will start in a state of RELAY OFF.
*   Only when the hub establishes that there is NO POWER in the generator cable will it instruct the relay devices to turn ON.
*   
*   
*   
* WIFI/SERIAL DEBUG FLAGS:
*   Control debug serial output gathered to a buffer.
*   If the WIFI_DEBUG flag is set this debug string is sent via the ESP-NOW message.
*   If SERIAL_DEBUG flag is set print using the Serial module.
*
* Serial / WiFi DEBUG: OUTPUT OPTION:
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
* ~~~~~~~~ ANALYSE ANTENNA OVER MANY CYCLES TO DETERMINE LIVE WIRE STATE ~~~~~~~~
* 
* OVERVIEW
*   With no isolator live OFF will give lower analysis value that live HOT!
* When live input out of phase with USB plug power value will range from very LOW to very HIGH!!!
* But MUST analyse over an extended period as phase may only drift slowly!
* 
* STATES
* 1..Power on for initial setup - MUST be on MAINS POWER generator live wire OFF.
*   Initialise triggered by reset - when GREEN LED ON during boot-up press PAIR btn.
*   Device will do first analysis and store as MAINS ON - GENERATOR OFF stable value.
*   This will be a STABLE LOWISH value eg 330 to 1200.
* 2..Mains fails - all devices power down.
* 3..Generator ON (mains off) all devices power up.
*   This will give a HIGH but STABLE value eg > 2000.
* 4..Mains power is restored and house switched to mains power - generator still on.
*   This will give UNSTABLW out-of-phase live result.
*   SWITCH DEVICES COULD TURN ON NOW?  
* 5..Generator power turned off.
*   Antenna shows stable lowish value.
*   SWITCH DEVICES COULD TURN ON NOW?  
* 6..Wait for back to 2 state.
* 
* NOTE: Determination of MAINS ON state is by the DIFFERENCE between the current antenna alalogue value and the REFERENCE value.
* A small difference means the condition of the monitored generator cable is CLOSE to the OFF reference value.
* A LARGE difference means the condition of the monitored generator cable is VERY DIFFERENT to the OFF reference value. SO LIVE WIRE ASSUMED HOT!!!
* 
* ANALYSIS ALGORITHM:
*   On initialise start flash GRN LED three times then do analysis to get REFERENBCE VALUE for mains on generator OFF.
* This REFERENCE VALUE say 1200 is stored in EEPROM and used as a basis for live wire state analysis.
* It SHOULD BE within a defined range say 800 to 1600. If so TURN GREEB LED ON for short period then enter RUN MODE. 
* ESLE RED LED ON enter FAULT MODE.
* Analysis loop records 20 readings at 1 mS intervals for ONE CYCLE.
* The mean of the MINIMUM of these over say 50 20 mS cycles is calcumated AND the VARIANCE vs the REFERENCE VALUE.
* If the variance is LOW then MAINS POWER is ON and the generator live wire is OFF.
* Otherwise the live wire IS HOT. (Steady HIGH value of very variable value).
* If a change in state is detected send a new status to switch devices.
* Every 3 seconds send a status to switch devices.
* 
* readOneCycle():
*   Loop 20 times with 1 mS delay and return lowest value found.
*   
* getMeanVariance():
*   Loop X times calling readOneCycle adding to mean total & variance total.
*   At end divide both by X and update global iMean & iVariance.
* 
* FLAG SETTING:
*   During boot after all LEDs flach twice...
* GRN LED ON - press btn will on grnBtnPressed flag. HOLD btn until GRN LED off set grnBtnHold flag.
* BLU LED ON - press btn will on bluBtnPressed flag. HOLD btn until BLU LED off set bluBtnHold flag.
* (Requires USB-OTG Mode)
* 
* 
* 
* ++++++++++++++++++++++  LOADING NEW CODE TO HUB & SWITCH DEVICES  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*   The HUB and SWITCH devices can be programmed with a new sketch using the Arduino IDE.
* The support for ESP32 and ESP8266 MCUs will need to be added.
* Instructions for this are available online eg: 
*    https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
*    https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/
*    
* ----------- HUB CONTROLLER --------------------------------------------------
* The HUB controller supports direct connection for sketch upload via a microUSB cable to DEV machine.
* However, auto management of the upload process is NOT supported at this time.
* Removal of the HUB controller lid may be neccessary (see step 4 below) to allow access to BOTH buttons (also for the FTDI interface if ever needed)
* STEPS-BY-STEP:
* 0..Select the correct device in the IDE: Tools -> Board: ESP32 Arduino -> ESP32S3 Dev Module.
*   AND FOLLOWING SETTING...
*   Upload Speed: "460800"                  << Use the fastest that works reliably. Default is 921600.>>
*   USB Mode: "USB-OTG (Tiny USB)"
*   USB CDC On Boot: "Enabled"              << This will allow Serial.print debug output - SEE NOTE BELOW! >>
*   USB Firmware MSC on Boot: "Disabled"    << Enables bin file sketch load. >>
*   USB DFU On Boot: Disabled               << Device Firmware Upgrade - Use to flash firmware. IF NEEDED!!! >>
*   Upload Mode: "USB-OTG CDC (Tiny USB)"   << Use mircoUSB rather than FTDI header. >>
* 1..Connect the HUB to the development PC/Laptop using a microUSB cable.
* 2..Ensure that a new serial port appears in the Tools -> Port: Menu.
* 3..Select this port.
* 4.. (This may not be required, can depend on the board and possible usb drivers etc, but included for completeness):
*   To uplaod a sketch hold down the PAIR button while pressing the RESET button.
*   This will place the HUB on "Waiting for download" mode.
* 5..Then press the UPLOAD icon on the IDE. The sketch will compile and then start loading.
* 6..After loading is complete press the reset button again.
* 7..During the boot process the GREEN LED will turn on for 3 seconds.
*   If the button is pressed during this time this will cause the HUB to calibrate for LIVE OFF on the generator cable.
*   After a 1 second delay the BLUE LED will turn on for 3 seconds.
*   If the button is pressed during THIS time this will cause the HUB to set the SERIAL DEBUG output flag.
*   When set thsi flag will cause the HUB to send debug data to the serial port to show on the IDE monitor window.
*   NOTE: If this flag is set while the HUB is powered from a USB plug i.e. with NO serial port the code WILL HANG!
*   PRESS THE BUTTON FOR BOTH OF THESE FLAG SETTING OPTIONS
* 8..Assuming the button was pressed for SERIAL DEBUG this data will be printed to the IDE monitor window.
*   
*   
* ========  Sample Debug Data on Boot  ========   
*   EEPROM_NUMPAIRED_ADDR = 2                             << TWO switch devices registered >>
*   de-init ESP-NOW: 1                                    << ESP-NOW init SUCCESS >>
*   MAC address for paired device # 1 0F:00:00:F2:9D:A3
*   MAC address for paired device # 2 AC:0B:FB:DB:C7:CF

*   ESP32-NOW_HUB_R1_STATS V1.2E                          << PROG_ID of this sketch >>
*   GET REF VAL = 4095                                    << Current generator live OFF calibration value >>
*   0F:00:00:F2:9D:A3                                     << MAC addr of first switch device >>
*   Send result: SUCCESS... ESP-NOW response time: 3
*   Delivery: GOOD
*   ----> Setup exiting...
*
*
* ========  Sample Loop Data  ========   
*   4095  4095  4095  4095  4095  4095  4095  4095   ....     << Sensor read values >>
*   Sensor min = 4095  max = 0                                << Min & Max sensor read values this loop >>
*   Sending to 2 paired devices...
*   Sending to device # 1 0F:00:00:F2:9D:A3
*   Send result: SUCCESS... ESP-NOW response time: 4          << Response delay in mS >>
*   Delivery: GOOD
*   Total success 7 / fail 0                                  << Analyse results over 100 sends to this switch device >>
*   Last 100 success 0 / fail 0                               << Ditto previous 100 sends to this device >>
*   Sending to device # 2 AC:0B:FB:DB:C7:CF
*   Send result: SUCCESS... ESP-NOW response time: 3
*   Delivery: GOOD
*   Total success 5 / fail 0
*   Last 100 success 0 / fail 0
*   
*   
* ---------- SWITCH DEVICES ----------------------------------
* IMPORTANT: This ESP8266 code for switch devices supports FOUR DIFFERENT hardwrae devices.
* There are FOUR DEFINE STATEMENTS in the code to facilitate this.
* THREE MUST BE COMMENTED OUT LEAVING ONE TO SPECIFY FOR WHICH HAREWARE THE CODE IS TO BE COMPILED.
* 
* #define ESP8266MOD2F
* // #define ESP01S
* // #define ESP01             // RED LED and BLUE
* // #define SONOFF            // For Sonoff mains wifi relay board.

* 
* The ESP8266MOD2F is very easy!
* 1..Connect using microUSB cable.
* 2..Select new port.
* 3..Select IDE menu Tools -> Board: ESP8266 Boards -> Generic ESP8266 Module.
* 4..Upload.
*   
* The ESP01 and ESP01S can be programmed using an FTDI header or Arduino UNO etc.
* There are many site available via Google to show this.
* Eg. https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/
* 
* Once connected proceed as above but REMEMBER to uncomment the CORRECT DEFINE in the code.
* Also, the GPIO 0 pin of the ESP8266 must be held at ground while reset pin grounded to start upload mode.
* 
* The SONOFF device requires an FTDI connected via a ribbon cable to the 4 pins on the device.
* WITH AC DISCONNECTED open the case and locate the header pins.
* GND, VCC, TX & Rx.
* Connect to the FTDI matching header pins using 4 core female/female jumper ribbon cable.
* NOTE: You MUST hold the button on the SONOF down WHILE CONNECTING THE FTDI to the dev PC/Laptop.
* There is NO RESET BUTTON. So this will ensure the device starts up in "waiting for download" mode.
* Then upload skecth in the usual way.
*   
*/

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <EEPROM.h>



#define PROG_ID "ESP32-NOW_HUB_R1_STATS V1.2G"  // MKI PCB - NO ISOLATOR - Use analysis of antenna inpt to determine live wire OFF / HOT. Mk1 PCB LED values!
// V1.1 - Clear pairing option if btn held.
// V1.2A - Rearrange setup. Mv initWFi. B: Fix debug print in setup. C: Update comments. D: Send 25 not 30 values in msg.  E: No clr pair on grn hold! Tidy commenst etc.
// V1.2F - Set ESP_NOW msg timeout 50 -> 100 mS! DEBUG_PRINT delay 500 -> 100. Loop delay. G: BACK TO 500!!

#define TICK_DELAY 100            // Duration of global loop delay. 100 mS = 1 "tick". 
#define MAC_SIZE_IN_BYTES 6       // A MAC address is 6 bytes long!
#define PAIRED_DEVICE_LIMIT 8     // How many paired devices supported.
#define MAC_ARRAY_SIZE (PAIRED_DEVICE_LIMIT * MAC_SIZE_IN_BYTES)  // Allow for up to all MAC addresses in array.
#define EEPROM_SIZE (MAC_ARRAY_SIZE + 4)          // Allow for MAC array, NUMPAIRED cold start flag and SERIAL_MODE_FLAG. And Live off ref value!
#define EEPROM_SERIAL_FLAG_ADDR 0 // NOT USED!!!
#define EEPROM_NUMPAIRED_ADDR 1   // Default value of EEPROM byte is 0xFF. Set location to num paired 0-8.
#define EEPROM_REFVAL_ADDR 2      //
#define EEPROM_MAC_ARRAY_ADDR 3   // Start address of MAC array.

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

#define SENSOR_DIFF 100             // ***ADJUST*** Analogue value variance from ref indicating live is HOT!
#define SENSOR_MAX 4095             // Analogue value when input at max.
#define SENSOR_TOP_TARGET 5         // Must hit this value to count a top of cycle.
#define SUM_CYCLES 50               // Analyse sensor input over this number of cycles.

#define MSG_COUNT_LIMIT 100     // Count up to this limit and analyes totals.
#define MSG_TIMEOUT_MS 100      // Wait this many mS for confirmed msg delivery. 50 mS is TOO SHORT. Actual delays were up to 50 mS!

#define DEBUG_DEVICE_INDEX 0    // This paired device is used for debug data messages.
#define DEBUG_MSG 0             // This message contains debug data to be printed.
#define MAINS_DATA_MSG 1        // This message contains mains status data to be used for device output control.
#define MSG_DELIVERY_PENDING 0    // Set by msg send calling code. Updated by callback fn.
#define MSG_DELIVERY_SUCCESS 1    // Got ACK from target device.
#define MSG_DELIVERY_FAILURE 2    // No ACK from target device.
#define TEXT_BUFFER_SIZE 200    // Use text buffers of this size AND CHK FOR OVER-RUN
#define DEBUG_VAL_LIMIT 25      // Send first 25 values in ESP-NOW msg.

bool SERIALPRINT = false;       // Prevents serial output on USB port blocking program execution. True on btn LED blue in boot!
bool WIFI_DEBUG = true;         // Do NOT use the ESP-NOW msg to print debug ouitput if false.
bool SERIAL_DEBUG = true;       // Do NOT use the Serial module to print debug output if false.
int loopCount = 0;              // Main loop counter.
bool btnPressed = false;        // Flag set by interrupt code on button pressed.
int lastMsgStatus = 0;          // Set by calling & callback fn. 0 = PENDING, 1 = GOOD, 2 = BAD.

bool grnBtnPressed = false;     // Set if btn was pressed WHILE GRN LED ON. SETS liveOff ref value.
bool grnBtnHeld = false;        // Set if btn was HELD until GRN LED OFF. Clears paired device array.
bool bluBtnPressed = false;     // Set flag if btn was pressed WHILE BLU LED ON. .
bool bluBtnHeld = false;        // Set flag if btn was HELD until BLU LED OFF. SETS DEBUG PRINT on flag.

int sensor_analog = 0;      // Value of analogue read.
int sensor_max = 4095;      // Max value read during one analysis.
int sensor_min = 0;         // Ditto min.
bool sensor_out = false;    // Value output by the sensor for mains detected true or not false.
int sensorCycleCount = 0;   // Count cycles during one analysis period.
int sensorIsMaxCount = 0;   // While sensor read is at max. Count when this happens.
int sensorIsMaxCountMax = 0;

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
int meanMin = 0;                              // Mean value of minimum sensor reading over analysis cycles.
int meanDiff = 0;                             // Mean value of difference between sensor reading and reference value for live OFF.
int liveOffRef = 0;                           // Use this value to find measure of difference from "LIVE OFF" sensor state.

char buffer[TEXT_BUFFER_SIZE];                // Working text buffer for debug print - DO NOT OVERFLOW!.

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
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(GPIO19, INPUT_PULLUP);
  pinMode(GPIO20, INPUT_PULLUP);
//  pinMode(GPIO9, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  digitalWrite(LED_RED, LED_OFF);  // Set initial state
  digitalWrite(LED_GRN, LED_OFF);
  digitalWrite(LED_BLU, LED_OFF);

  // Set up button interrupt on GPIO pin.
  attachInterrupt(BTN_PIN, buttonInt, FALLING);  // Pin has pull-up res. LOW when pressed.
  btnPressed = false;

  multiBlinkLED(LED_ALL, BLINK_DELAY_FAST, 2); // Show sign of life!

  EEPROM.begin(EEPROM_SIZE);

  // Check for button pressed during boot - if so set LIVE OFF reference value..
  // GREEN..........
  digitalWrite(LED_GRN, LED_ON);
  delay(3000);
  if (digitalRead(BTN_PIN) == BTN_DOWN) {
    grnBtnHeld = true;
  }
  else if (btnPressed) {
    grnBtnPressed = true;
  }
  btnPressed = false;
  digitalWrite(LED_GRN, LED_OFF);
  delay(1000);
  
  // BLUE..........
  digitalWrite(LED_BLU, LED_ON);
  delay(3000);
  if (digitalRead(BTN_PIN) == BTN_DOWN) {
    bluBtnHeld = true;
  }
  else if (btnPressed) {
    bluBtnPressed = true;
  }
  btnPressed = false;
  digitalWrite(LED_BLU, LED_OFF);
  delay(1000);

  if (bluBtnPressed) {
    multiBlinkLED(LED_BLU, BLINK_DELAY_FAST, 3); // Show btn pressed!
    SERIALPRINT = true;     // Set flag to see serial output - MUST BE USB TO PC/LAPTOP or HANG!!!!
  }

  // Init Serial Monitor
  if (SERIALPRINT) Serial.begin(115200);
  if (SERIALPRINT) while (!Serial) {;}
  

  if (SERIALPRINT) Serial.printf("EEPROM_NUMPAIRED_ADDR = %i\n", EEPROM.read(EEPROM_NUMPAIRED_ADDR)); // Show stored array size.

  if (EEPROM.read(EEPROM_NUMPAIRED_ADDR) != 0xFF) {
    // This is NOT a cold start so read MAC address arrary from EEPROM to RAM...
    readEEPROMArray();  // Includes pairedItemCount.
  }

  initWiFi();
  delay(100);   // Arbitrary wifi ready delay ????

  // Set sensor input LIVE OFF reference value.
  if (grnBtnPressed) {
    multiBlinkLED(LED_GRN, BLINK_DELAY_FAST, 3); // Show btn pressed!
    getMeanVariance();
    sprintf(buffer, "SET REF VAL = %i\n", meanMin);
    DEBUG__PRINT(SEND);
    EEPROM.writeInt(EEPROM_REFVAL_ADDR, meanMin);
    EEPROM.commit();
  }
  liveOffRef = EEPROM.readInt(EEPROM_REFVAL_ADDR);
  
  clearMsgAnalysis(true);
  
  sprintf(buffer, "\n\n%s\n", PROG_ID);
  DEBUG__PRINT(APPEND);
  sprintf(buffer, "GET REF VAL = %i\n", liveOffRef);
  DEBUG__PRINT(SEND);

  sprintf(buffer, "%s\n", " ----> Setup exiting...");    // MISSING ON REMOTE DEVICE... W H Y ? ? 
  DEBUG__PRINT(SEND);

//  EEPROM.end();
}

// Find min analog value in one 50 Hz cycle.
int readOneCycle() {
  int iSensorMin = 4095; 
  int iSensorAnalog = 0;
  for (int i = 0; i < 20; i++) {
    iSensorAnalog = analogRead(SENSOR_IN);
    // Find min value in this cycle...
    if (iSensorAnalog < iSensorMin) {
      iSensorMin = iSensorAnalog;
    }
    delay(1);
  }
  return iSensorMin;
}

// Calculate mean min value of V = sum V / num items. Variance (from ref value) sum (ref - V)^2
void getMeanVariance() {
  long total = 0;   // To calc mean
  long diff = 0;    // To calc mean of total diff from ref.
  int cycleMin = 0; // Min value found in one cycle.
  for (int i = 0; i < SUM_CYCLES; i++) {
    cycleMin = readOneCycle();
    total += cycleMin;
    diff += abs(cycleMin - liveOffRef);
  }
  meanMin = total / SUM_CYCLES;
  meanDiff = diff / SUM_CYCLES;
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
    // Find min value in this period...
    if (sensor_analog < sensor_min) {
      sensor_min = sensor_analog;
    }
    delay(1);
    // #### Add first DEBUG_VAL_LIMIT values to msgBuf #### AVOID OVERRUN!
    if (i < DEBUG_VAL_LIMIT) {
      sprintf( (myData.espChars + (i * 5)), "%04i ", sensor_analog);
    }
  }
  sprintf( (myData.espChars + (DEBUG_VAL_LIMIT * 5)), "Min: %04i  MaxCnt: %04i", sensor_min, sensorIsMaxCountMax);

  getMeanVariance();
  sprintf( (myData.espChars + (DEBUG_VAL_LIMIT * 5) + 23), "\nmeanMin: %04i  Diff: %04i \n", meanMin, meanDiff);
  
  // ANALYSE the signal detected send results accordingly.
  if ( (meanDiff > SENSOR_DIFF) ) {
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
