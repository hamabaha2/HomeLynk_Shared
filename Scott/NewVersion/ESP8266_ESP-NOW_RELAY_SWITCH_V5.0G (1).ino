/*
    Mains Detector System SWITCH DEVICE

  OVERVIEW:
   This device listens for ESP-NOW messages from it's HUB controller.
   The HUB has a sensor to detect presence or absence of a HOT live core in a cable.
   An ESP-NOW message containing the current state of the LIVE wire detector is sent at regular invervcals.
   This interval is defined by LOOP_EXEC_DELAY_SECS on the HUB and check at this frequency on this device..
   Only devices with their MAC address registered with the hub "PAIRED" will be sent messages addressed to their MAC.
   Devices are "PAIRED" by putting BOTH the HUB and the SWITCH device into "PAIRING MODE".
   A button press on each device shorting GPIO0 to ground will invoke "PAIRING" mode.
   On MODULE devices with no button a RESET button press during start up while BLUE LED on steady will invoke PAIRING.
   Once the HUB is waiting for PAIRING the SWITCH/RELAY device should invoke "PAIRING" mode.
   The HUB enters WiFi AP mode and the SWITCH enters AP_STA mode and attempts to join the WiFi with specific SSID & password.
   When the SWITCH has successfully joined the WiFi network it resets to exit pairing mode and run active mode.
   When the HUB has successfully accepted the SWITCH into it's AP clients it too exits pairing mode.
   But AFTER updating it's MAC address list in EEPROM.
   Up to 8 SWITCH devices can pair with a HUB.

  SUPPORTED MODULES:
   This code can be compiled to support four different ESP8266 dev modules:
  1..ESP8266 ESP-12F WeMos D1 Mini - 16 pin modules with interated side reset button.
  2..ESP-01S - tiny 8 pin module which can plug into a relay module. Needs adapter for breadboard use.
  3..ESP01 - almost identical to ESP-01S but with TWO LEDs. Red is just power on. Blue to Tx pin GPIO 1.
  4..SONOFF Basic - Single relay mains control unit. Can be easily opened and has PTH holes for pin header.

  Controlled by define below. ONE DEF ONLY!!! Comment others.
  ESP8266-MOD2F - Small 16-pin breadboard capable nmodule. One blue LED (GPIO2) & reset switch on side.
  ESP01S - Tiny 8 pin non-breadboard module. One blue LED (GPIO2).
  ESP01 - Similar to above. But with TWO LED red (power on) and blue (GPIO1 - Tx) so NO SERIAL OUT!
    Both the above the above modules can be used with a relay board.
    But ESP01 device will HANG if the Serail port print is attempted!!!
    !!!! So print statements excluded by "ifdef" to avoid problem !!!!
  Sonoff "Basic" - Mains relay module. One blue / multicolour LED (GPIO13). Relay GPIO12. Button GPIO0.
    No reset sw so to flash, power up with button pressed.

  RELAY CONTROL:
    The realy should ONLY ENERGISE when the generator LIVE state IS OFF.
  This is to ensure the household load is not too great for the generator.
  So the initial power-on state of the logic is GENERATOR POWER IS ON!
  Only when a confirming signal that LIVE is OFF should the relay energise.

    
  IMPORTANT: The inconsistency of these board means YOU MUST UNCOMMENT THE CORRECT DEVICE  DEFINE STATEMENT!!!!!!!

  RELAY MODULE:
   See defines below for different module outputs.

  ESP-01 vs ESP-01S MODULES:
   ESP-01 has TWO LEDs
     A red connected to VCC & GND to indicate it is powered.
     B blue connected to it's Tx Pin GPIO 1

  DEVICE BLUE LED usage:
    OFF:  Powered Down
    ON for short double blink mostly OFF: Connected and LIVE core is OFF
    OFF for short double blink mostly ON: Connected and LIVE core is ON/HOT
    Continuous fast blink: PAIRING.
    Continuous slow blink: NOT CONNECTED.

  RELAY BOARD LOGIC
   As the relay module uses GPIO0 to power the relay AND relay board LED we CANNOT use that for pairing button.
   However, there IS a RESET button on the relay module. We can use this to facilitate pairing as follows:
  1..We do NOT want the SWITCH device to attempt pairing on power-up.
  2..We DO want it to invoke pairing when the RESET button is pressed.
  So, on boot read EEPROM PAIR location. if this is 0xFF this is a NORMAL BOOT.
  After ~ 2 seconds set PAIR LOCATION to 55 and turn on BLUE LED.
  IF reset button pressed now...when we read EEPROM PAIR location it is 0x55 to enter PAIR MODE.
  If reset button is NOT pressed within 2 more seconds while LED is on then set PAIR LOCATION to 0xFF.
  Then proceed to run code as normal.

  DEBUG:
    Message struct has a byte type: 0 = DEBUG, 1 = LIVE cable Status.

  NOTE:
    During sketch UPLOAD on Arduino IDE MAC addres of device is listed eg: MAC: 48:55:19:16:5b:f2
*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

#define PROG_ID "ESP8266-NOW_SWITCH V5.0M" // Multi Module defs.
// 3.1: Add message reading stability logic.
// 3.2: Change PAIRING command from BTN 0 input to reset timing - as above. A: Adjust pair flag logic. B: Move WiFi STA.
//  C: WiFi.begin!  D: setup order! E; Strng "test"  F: MAC addr. G: Com TO 5->9  DELAY aft pair. H: Power RELAY!  J: 160 chars.
//  I: RELAY_PIN output.  J: Fix stability!
// 3.3: Move RSSI from Rx loop to pairing. Relay off on boot.
//  A: Move print of new data from callback to mainLoop.
//  B: Tidy comments NO EXECUTION CHANGES.
// 3.4: Final code walk through & tidy / fix!
//  A: Change structure! B: STA mode! C: Connect TO -> 60  D: Reset EEPROM flag at START of fn. Use WIFI-AP-STA mode!
//  E: WiFi.persistent(false);  F: Add DEBUG define for ESP-01 vs S module (S has one LED). G: 120 Char struct.
// 5.0 New struct. A: Multi-module defs.  B: Bnt 3  C: Relay on/off logic per device!  D: Abort loop delay.  E: Comments minimal tidy.
//  F: Use DEBUG__PRINT everywhere.
//  G: FIX Print MAC address.
//  H: FIX buffer size 100 WAY TOO SMALL -> 250.
//  J: Relay logic generator LIVE HOT => RELAY OFF!
//  K: Default LIVE cable state is ON
//  L: Set BTN_PIN input pullup and NO INTERRUPT on ESP01
//  M: Comment edit only.

#define ESP8266MOD2F
// #define ESP01S
// #define ESP01             // RED LED and BLUE
// #define SONOFF            // For Sonoff mains wifi relay board.

#ifdef ESP8266MOD2F       // Use GPIO2 for LED and Serial debug ON. Else use Tx pin GPIO1 for LED and NO DEBUG PRINT
#define BTN_PIN 0       // Pin to select pairing mode control - IF USED. Reset button also works.
#define LED_BLU 2       // NOTE: This is LED_BUILTIN and LOW is ON, HIGH is OFF.
#define RELAY_PIN 4     // Pin to power relay if used. LOW is ON.
#define RELAY_OFF 1     // On / off logic default 0 ON.
#define RELAY_ON 0
#endif


#ifdef ESP01S             // Use GPIO2 for LED and Serial debug ON. Else use Tx pin GPIO1 for LED and NO DEBUG PRINT
#define BTN_PIN 3       // Pin to select pairing mode control - IF USED. Reset button also works.
#define LED_BLU 2       // NOTE: This is LED_BUILTIN and LOW is ON, HIGH is OFF.
#define RELAY_PIN 0     // Pin to power relay if used. LOW is ON.
#define RELAY_OFF 1     // On / off logic default 0 ON.
#define RELAY_ON 0
#endif


#ifdef ESP01
#define BTN_PIN 3       // Pin to select pairing mode control - IF USED. Reset button also works.
#define LED_BLU 1      // NOTE: This is LED_BUILTIN and LOW is ON, HIGH is OFF. This is Tx pin - NO SERIAL!
#define RELAY_PIN 0    // Pin to power relay if used. LOW is ON.
#define RELAY_OFF 1     // On / off logic default 0 ON.
#define RELAY_ON 0
#endif

#ifdef SONOFF
#define BTN_PIN 0       // Pin to select pairing mode control - IF USED. Reset button also works.
#define LED_BLU 13      // NOTE: This is LED_BUILTIN and LOW is ON, HIGH is OFF.
#define RELAY_PIN 12    // Pin to power relay if used. LOW is ON.
#define RELAY_OFF 0     // On / off logic reversed!
#define RELAY_ON 1
#endif


#define LOOP_EXEC_DELAY_SECS 3    // Execute main loop code every 3 seconds
#define PAIRING_TIMEOUT_SECS 60   // Limit pairing mode time during setup.
#define PAIRING_MODE_FLAG 0x55    // Pairing mode flag. TRUE if set to this value else FALSE.
#define PAIRING_DELAY 3000        // Wait this long for user to reset for pairing mode.
#define EEPROM_SIZE 64            // Declare size needed.
#define EEPROM_PAIRING_ADDR 0     // Location to store EEPROM pairing flag value.
#define COMMS_TIMEOUT_SECS 9      // haveComm = false if no mesage for this time.
#define NO_COMMS_MAX 999          // Count ticks while no comms.
#define NO_CHANGE_MAX 999         // Count ticks while no change.
#define MIN_STABLE_COUNT 2        // Only set new LIVE on / off state if have this many CONSECUTIVE MATCHING VALUES.
#define WIFI_SSID "ESP32"
#define WIFI_PASS "magicword"

int loopCount = 0;                // Main code loop counter - LED timing etc based on this.
int noCommsCount = NO_COMMS_MAX;  // Count while no comms message received up to NO_COMMS_MAX.
bool btnPressed = false;          // Set by button interrupt.
bool haveComms = false;           // Set true when ESP-NOW message received. False when no message for COMMS_TIMEOUT
bool liveIsHOT = true;           // Set by OnDataRecv fn TRUE if message says LIVE is ON/HOT. Used to control blue default LED.
bool currentLiveIsHotValue = true; // Record current value for stability analysis.
bool lastLiveIsHotValue = true;    // Save value of last liveIsHOT flag to count stability
int noChangeCount = 0;            // How stable is this liveIsHOT reading? Inc if current == last liveIsHOT Value. Else ZERO it!
bool newData = false;             // Flag new data in callback fn. Avoid Serial.print here.
int newDataLen = 0;               // Update in callBack fn for use by main loop logic.
char buffer[250];                 // Working text buffer for debug print - DO NOT OVERFLOW!.

// Structure example to receive data.
// Must match the sent structure.
typedef struct esp_message {
  byte espType;           // 0 => DEBUG. 1 => Live cable state data.
  bool espMains;          // If type == 1 this is LIVE cable state false = OFF, true = ON.
  char espChars[200];     // Text message DEBUG serial out.
};

// Create a struct_message called myData
esp_message myData;

void DEBUG__PRINT(const char text[]) {
#ifndef ESP01               // Only this module of the set supported uses the Tx pin for LED so NO PRINT POSSIBLE!!!
  Serial.print(text);
  Serial.flush();
#endif
}
void DEBUG__PRINTLN(const char text[]) {
#ifndef ESP01
  Serial.println(text);
  Serial.flush();
#endif
}


// When button pressed. Wait until released and react based on LONG or SHORT press.
// NOTE: "IRAM_ATTR" needed to prevent "ISR not in IRAM" error!!!
void IRAM_ATTR buttonInt() {
  btnPressed = true;
}



// ==== Handle pairing mode ====
// May be invoked by buttonn press OR during double reset sequence logic.
// This will attempt to join the HUB AP which will expose this device MAC address to the HUB.
// The HUB must also ALREADY BE in pairing mode. So will add this MAC address to paired array.
void pairingMode() {
  int count = 0;
  DEBUG__PRINT("Pairing mode...\n");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  DEBUG__PRINT("Connecting to WiFi ..");
  DEBUG__PRINTLN(WIFI_SSID);

  // Wait for successful connection to AP or time-out...
  while ( (count < (PAIRING_TIMEOUT_SECS * 10)) && (WiFi.status() != WL_CONNECTED) ) {
    count++;
    digitalWrite(LED_BLU, !digitalRead(LED_BLU));   // Rapid blink LED while waiting...
    delay(100);
  }
  sprintf(buffer, "Loop count was %i\n", count);  //  Check how long it took!
  DEBUG__PRINT(buffer);
  if (count >= PAIRING_TIMEOUT_SECS * 10) {
    // No connection - timed out!
    DEBUG__PRINT("Connection FAILED - Timed out!!!");
    digitalWrite(LED_BLU, LOW);                     // LED ON for 2 secs if failed to connect.
    delay(2000);
  } else {
    DEBUG__PRINTLN("Connection SUCCESS!!!");
    delay(3000);                                        // Let HUB see connection B4 reset. THIS DELAY IS NEEDED!!!
    ESP.restart();                                      // Once we have a connection reboot the module. #### TO REVEIW!!! ####
  }
}


// Handle pairing button pressed action.
// Count loop ticks to establish...
// 1..Debounce.  2..Valid short button press.
// NOTE: Loops FOREVER while button  held down!
void btnDown() {
  int count = 0;
  //==== Button DOWN Logic - DEBOUNCE =================================================================================
  while ( (digitalRead(BTN_PIN) == 0) || (count < 5) ) {
    count++;
    delay(100);
  }
  if (count > 5) {
    pairingMode();
  } else {
    //==== Button UP Logic - INVALID =====================================================================================
    DEBUG__PRINTLN("Button press TOO SHORT!");
  }
  // Clear flag - this button event has been handled.
  btnPressed = false;
}



// Callback function that will be executed when data is received...
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  newDataLen = len;
  int copyLen = min(uint(len), sizeof(myData));   // Never copy more daya than can fit on struct!
  memcpy(&myData, incomingData, copyLen);
  // If this is a data message SET/CLEAR THE LIVE STATE FLAG...
  if (myData.espType == 1) {
    currentLiveIsHotValue = myData.espMains;

    // Count up to a MAX value consecutive matching current/last liveIsHOT on/off reports.
    if ( (lastLiveIsHotValue == currentLiveIsHotValue) && (noChangeCount < NO_CHANGE_MAX) ) {
      noChangeCount++;
    }
    // Set liveIsHOT flag to what has just been received IF value is "STABNLE" enough. Else leave unchanged.
    liveIsHOT = noChangeCount < MIN_STABLE_COUNT ? liveIsHOT : currentLiveIsHotValue;

    lastLiveIsHotValue = currentLiveIsHotValue;       // Save for stability check
    noCommsCount = 0;                             // Clear count - we have a message!
  }
  haveComms = true;                             // Set comms flag.
  newData = true;                               // Set flag to indicate we have a new data message.
  loopCount = (LOOP_EXEC_DELAY_SECS * 10) + 1;  // Abort loop delay and process new message immediatly.
}

void setup() {
  // Initialize Serial Monitor
#ifndef ESP01
  Serial.begin(115200);
#endif
  delay(1000);
  DEBUG__PRINTLN("\n\n");
  DEBUG__PRINTLN(PROG_ID);
  DEBUG__PRINT("Starting...");

  pinMode(BTN_PIN, INPUT_PULLUP);     // Initialize as an intput.
  pinMode(LED_BLU, OUTPUT);           // Initialize the LED_BUILTIN pin as an output.
  pinMode(RELAY_PIN, OUTPUT);         // Initialize the RELAY power control pin as an output.
  digitalWrite(RELAY_PIN, RELAY_OFF); // Ensure RELAY is NOT energised on boot.

  // Toggle LED 3 times to show device is in setup.
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BLU, LOW);
    delay(100);
    digitalWrite(LED_BLU, HIGH);
    delay(100);
  }

  // Set device as a Wi-Fi AP / Station and prind details...
  WiFi.mode(WIFI_AP_STA);
  sprintf(buffer, "RCVR Channel: %i\n", WiFi.channel());
  DEBUG__PRINT(buffer);

  #ifndef ESP01
    Serial.print("ESP Board MAC Address: ");
    Serial.println(WiFi.macAddress());
  #endif

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    DEBUG__PRINTLN("Error initializing ESP-NOW");
    return;   // ??? NOT SURE WHY??? REBOOT after delay???
  }

  // Once ESPNow is successfully Init, we will register for recv CB.
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  #ifndef ESP01
  // Set up button interrupt on GPIO 0 pin.
  attachInterrupt(BTN_PIN, buttonInt, FALLING);  // Pin has pull-up res. LOW when pressed.
  #endif

  EEPROM.begin(EEPROM_SIZE);
  // Check EEPROM to see if this is a double reset pairing event...
  if (EEPROM.read(EEPROM_PAIRING_ADDR) == PAIRING_MODE_FLAG) {
    // Turn off pairing mode flag so reset will abort - otherwise we get back here!
    EEPROM.write(EEPROM_PAIRING_ADDR, 0xFF);
    EEPROM.commit();
    DEBUG__PRINT("EEPROM_PAIRING_ADDR, 0xFF\n");
    pairingMode();
  } else {
    // Set the pairing flag...
    EEPROM.write(EEPROM_PAIRING_ADDR, PAIRING_MODE_FLAG);
    EEPROM.commit();
    DEBUG__PRINTLN("EEPROM_PAIRING_ADDR, PAIRING_MODE_FLAG");
  }
  digitalWrite(LED_BLU, LOW);
  delay(PAIRING_DELAY);  // !!!!! IF RESET NOW WILL INVOKE PAIRING MODE!!!!!
  // Recind option to pair!!!
  EEPROM.write(EEPROM_PAIRING_ADDR, 0xFF);
  EEPROM.commit();
  DEBUG__PRINTLN("EEPROM_PAIRING_ADDR, 0xFF");

  myData.espMains = 1;    // Default state is LIVE cable is HOT - so relay de-eneggised to start.
  
  // Continue to normal operation. If RESET button was pressed in time then restart will invoke pairing mode!!!
}


// Execute every LOOP_EXEC_DELAY_SECS to manage the LED showing LIVE cable state and comms status.
// NOTE: If a relay is to be controlled it will be in this function.
void mainLoopCode() {
  // If we have new data message prinft details. Moved from inside callback fn.
  if (newData) {
    sprintf(buffer, "Bytes received: %i\n", newDataLen);
    DEBUG__PRINT(buffer);
    if (myData.espType == 1) {
      sprintf(buffer, "LIVE: %i\n", myData.espMains);
      DEBUG__PRINT(buffer);
    }
    DEBUG__PRINT("HUB:: ");
    DEBUG__PRINTLN(myData.espChars);
    newData = false;
  }
  // Adjust LED blink pattern for comms / no-comms AND if comm LIVE state.
  if (haveComms) {
    if (liveIsHOT) {
      digitalWrite(LED_BLU, LOW);
      // #### ADD ANY SWITCH DEVICE CONTROL HERE ####
      digitalWrite(RELAY_PIN, RELAY_OFF);
    } else {
      digitalWrite(LED_BLU, HIGH);
      // #### ADD ANY SWITCH DEVICE CONTROL HERE ####
      digitalWrite(RELAY_PIN, RELAY_ON);
    }
    // Toggle LED to show device is active: Two flashes to OFF if liveIsHOT, to ON if LIVE os OFF.
    for (int i = 0; i < 2; i++) {
      digitalWrite(LED_BLU, !digitalRead(LED_BLU));
      delay(100);
      digitalWrite(LED_BLU, !digitalRead(LED_BLU));
      delay(100);
    }
  } else {
    // NO COMMS - just alternate LED state at main loop execute frequency.
    digitalWrite(LED_BLU, !digitalRead(LED_BLU));
  }
}

// BASIC CODE LOOP - frequency 10 times per second - oncve per "TICK" i.e. 100mS
void loop() {
  loopCount++;
  // Manage main loop calls...
  if (loopCount > (LOOP_EXEC_DELAY_SECS * 10) ) {
    loopCount = 0;
    // Execute main loop code once every LOOP_EXEC_DELAY_SECS
    mainLoopCode();
  }
  // Manage comms fail status...Increment up to max.
  if (noCommsCount < NO_COMMS_MAX) {
    noCommsCount++;  // Stop at max. Reset on message received.
  }
  if (noCommsCount == (COMMS_TIMEOUT_SECS * 10) ) {
    haveComms = false;    // At the threshold clear the flag.
  }
  // Handle button pressed event...
  if (btnPressed) {
    btnDown();
  }
  // Delay for a "tick"...
  delay(100);
}
