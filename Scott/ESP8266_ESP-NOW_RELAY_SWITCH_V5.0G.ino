/*
*   Mains Detector System SWITCH DEVICE
*
* OVERVIEW:
*  This device listens for ESP-NOW messages from it's HUB controller.
* The HUB has a sensor to detect presence or absence of MAINS POWER.
* An ESP-NOW message containing the current state of the mains detector is sent at regular invervcals.
* This interval is defined by LOOP_EXEC_DELAY_SECS.
* Only devices with their MAC address registered with the hub "PAIRED" will be send messages addressed to their MAC.
*  Devices are "PAIRED" by putting BOTH the HUB and the SWITCH device into "PAIRING MODE".
*  A button press on each device shorting GPIO0 to ground will invoke "PAIRING" mode.
*  On RELAY MODULE devices a RESET button press during start up while BLUE LED on steady will invoke PAIRING.
*  Once SWITCH is waiting for PAIRING a button shorting GPIO0 to ground on the HUB will invoke "PAIRING" mode.
* The HUB enters WiFi AP mode and the SWITCH enters AP mode and attempts to join the WiFi with specific SSID & password.
* When the SWITCH has successfully joined the WiFi network it resets to exit pairing mode and run active mode.
* When the HUB has successfully accepted the SWITCH into it's AP clients.
* Then it too resets after updating it's MAC address list to EEPROM.
* Up to 6 SWITCH dvices can pair with a HUB.
* 
* RELAY MODULE:
*  #define RELAY 0 // relay connected to  GPIO0.

*
* ESP-01 LED usage:
*   OFF:  Powered Down
*   ON for short double blink mostly OFF: Connected and mains power is OFF
*   OFF for short double blink mostly ON: Connected and mains power is ON
*   Continuous fast blink: PAIRING.
*   Continuous slow blink: NOT CONNECTED.
*
* RELAY BOARD LOGIC
*  As the relay module used GPIO0 to power the relay AND relay board LED we CANNOT use that for pairing button.
* However, there IS a RESET button on the relay module. We can use this to facilitate pairing as follows:
* 1..We do NOT want the SWITCH device to attempt pairing on power-up.
* 2..We DO want it to invoke pairingh when the RESET button is pressed.
* So, on boot read EEPROM PAIR location. if this is 0xFF this is a NORMAL BOOT.
* After ~ 2 seconds set PAIR LOCATION to 55 and turn on LED.
* IF reset button pressed now...when we read EEPROM PAIR location is is 0x55 to enter PAIR MODE.
* If reset button is NOT pressed within 2 more seconds while LED is on then set PAIR LOCATION to 0xFF.
* Then proceed to run code as normal.
* 
* NOTE:
*   During sketch UPLOAD on Arduino IDE MAC addres of device is listed eg: MAC: 48:55:19:16:5b:f2
*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

#define PROG_ID "ESP8266-NOW_SWITCH V3.3B" // Tidy code from fully functioning device...
// 3.1: Add message reading stability logic.
// 3.2: Change PAIRING command from BTN 0 input to reset timing - as above. A: Adjust pair flag logic. B: Move WiFi STA.
//  C: WiFi.begin!  D: setup order! E; Strng "test"  F: MAC addr. G: Com TO 5->9  DELAY aft pair. H: Power RELAY!  
//  I: RELAY_PIN output.  J: Fix stability!
// 3.3: Move RSSI from Rx loop to pairing. Relay off on boot.
//  A: Move print of new data from callback to mainLoop.
//  B: Tidy comments NO EXECUTION CHANGES.

#define LOOP_EXEC_DELAY_SECS 3    // Execute main loop code every 3 seconds
#define BTN_PIN 2                 // Pin to select pairing mode control - IF USED. Reset button also works.
#define RELAY_PIN 0               // Pin to power relay if used. LOW is ON. HOLD TO GND TO FLASH!
#define LED_BLU 2                 // NOTE: This is LED_BUILTIN and LOW is ON, HIGH is OFF.
#define PAIRING_TIMEOUT_SECS 30   // Limit pairing mode time during setup.
#define PAIRING_MODE_FLAG 0x55    // Pairing mode flag. TRUE if set to this value else FALSE.
#define EEPROM_SIZE 64            // Declare size needed.
#define EEPROM_PAIRING_ADDR 0     // Location to store EEPROM pairing flag value.
#define COMMS_TIMEOUT_SECS 9      // haveComm = false if no mesage for this time.
#define NO_COMMS_MAX 999          // Count ticks while no comms.
#define NO_CHANGE_MAX 999         // Count ticks while no change.
#define MIN_STABLE_COUNT 2        // Only set new mains on / off state if have this many CONSECUTIVE MATCHING VALUES.
#define WIFI_SSID "ESP32"
#define WIFI_PASS "magicword"

int loopCount = 0;                // Main code loop counter - LED timing etc based on this.
int noCommsCount = NO_COMMS_MAX;  // Count while no comms message received up to NO_COMMS_MAX.
bool btnPressed = false;          // Set by button interrupt.
bool haveComms = false;           // Set true when ESP-NOW message received. False when no message for COMMS_TIMEOUT
bool mainsIsOn = false;           // Set by OnDataRecv fn TRUE if message says mains is ON. Used to control blue default LED.
bool currentMainsOnValue = false; // Record current value for stability analysis.
bool lastMainsOnValue = false;    // Save value of last mainsIsOn flag to count stability
int noChangeCount = 0;            // How stable is this mainsIsOn reading? Inc if current == last MainsOnValue. Else ZERO it!
bool newData = false;             // Flag new data in callback fn. Avoid Serial.print here.
int newDataLen = 0;               // Update in callBack fn for use by main loop logic.

// Structure to receive data
// Must match the sender structure
// #### MORE COMPLEX THAN NEEDED!!! ####
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  String d;
  bool e;
};

// Create a struct_message called myData
struct_message myData;


// ==== Handle pairing mode ====
// May be invoked by buttonn press OR during double reset sequence logic.
// This will attempt to join the HUB AP which will expose this device MAC address to the HUB.
// The HUB must also ALREADY BE in pairing mode. So will add this MAC address to paired array.
void pairingMode() {
  int count = 0;
  Serial.println("Pairing mode...\n");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());            // NOTE: This always reads 31!?
  Serial.print("Connecting to WiFi ..");
  Serial.println(WIFI_SSID);
  
  // Wait for successful connection to AP or time-out...
  while ( (count < (PAIRING_TIMEOUT_SECS * 10)) && (WiFi.status() != WL_CONNECTED) ) {
    count++;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // Rapid blink LED while waiting...
    delay(100);
  }
  Serial.printf("Loop count was %i\n", count);        //  Check how long it took!
  // Turn off pairing mode flag...
  EEPROM.write(EEPROM_PAIRING_ADDR, 0xFF);
  EEPROM.commit();   
  Serial.println("EEPROM_PAIRING_ADDR, 0xFF");
  if (count >= PAIRING_TIMEOUT_SECS * 10) {
    // No connection - timed out!
    Serial.print("Connection FAILED - Timed out!!!");
    digitalWrite(LED_BUILTIN, LOW);                     // LED ON for 2 secs if failed to connect.
    delay(2000);
  } else {
    Serial.print("Connection SUCCESS!!!");
    delay(3000);                                        // Let HUB see connection B4 reset. THIS DELAY IS NEEDED!!!
    ESP.restart();                                      // Once we have a connection reboot the module. #### TO REVEIW!!! ####
  }
}


// Callback function that will be executed when data is received...
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  // SET/CLEAR THE MAINS STATE FLAG...
  if (myData.b < 10) {
    currentMainsOnValue = false;
  } else {
    currentMainsOnValue = true;
  }
  // Count up to a MAX value consecutive matching current/last mains on/off reports.
  noChangeCount = (lastMainsOnValue == currentMainsOnValue) && (noChangeCount < NO_CHANGE_MAX) ? noChangeCount + 1 : noChangeCount; 
  // Set MainsIsOn flag to what has just been received IF value is "STABNLE" enough. Else leave unchanged.
  mainsIsOn = noChangeCount < MIN_STABLE_COUNT ? mainsIsOn : currentMainsOnValue;
  
  lastMainsOnValue = currentMainsOnValue;       // Save for stability check
  noCommsCount = 0;                             // Clear count - we have a message!
  haveComms = true;                             // Set comms flag.
  newData = true;                               // Set flag to indicate we have a new data message.
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n");
  Serial.println(PROG_ID);
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output.
  pinMode(RELAY_PIN, OUTPUT);       // Initialize the RELAY power control pin as an output.
  digitalWrite(RELAY_PIN, HIGH);    // Ensure RELAY is NOT energised on boot.

  // Toggle LED 3 times to show device is in setup.
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }

  // Set device as a Wi-Fi Station and prind details...
  WiFi.mode(WIFI_STA);
  Serial.print("RCVR Channel: ");
  Serial.println(WiFi.channel());
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;   // ??? NOT SURE WHY??? REBOOT after delay???
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  EEPROM.begin(EEPROM_SIZE);
  // Check EEPROM to see if this is a double reset pairing event...
  if (EEPROM.read(EEPROM_PAIRING_ADDR) == PAIRING_MODE_FLAG) {
    pairingMode();
  } else {
    // Set the pairing flag...
    EEPROM.write(EEPROM_PAIRING_ADDR, PAIRING_MODE_FLAG);
    EEPROM.commit(); 
    Serial.println("EEPROM_PAIRING_ADDR, PAIRING_MODE_FLAG");
    
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);  // !!!!! IF RESET NOW WILL INVOKE PAIRING MODE!!!!!
  // Recind option to pair!!!
  EEPROM.write(EEPROM_PAIRING_ADDR, 0xFF);
  EEPROM.commit(); 
  Serial.println("EEPROM_PAIRING_ADDR, 0xFF");
  
  // Continue to normal operation. If RESET button was pressed in time then restart will invoke pairing mode!!!
}


// Execute every LOOP_EXEC_DELAY_SECS to manage the LED showing mains state and comms status.
// NOTE: If a relay is to be controlled it will be in this function.
void mainLoopCode() {
  // If ew have new data message prinft details. Moved from inside callback fn.
  if (newData) {
    Serial.print("Bytes received: ");
    Serial.println(newDataLen);
    Serial.print("Char: ");
    Serial.println(myData.a);
    Serial.print("Int: ");
    Serial.println(myData.b);
    Serial.print("Float: ");
    Serial.println(myData.c);
    //  Serial.print("String: ");
    //  Serial.println(myData.d);
    Serial.print("Bool: ");
    Serial.println(myData.e);
    Serial.println();    
    newData = false;
  }
  // Adjust LED blink pattern for comms / no-comms AND if comm mains state.
  if (haveComms) {
    if (mainsIsOn) {
      digitalWrite(LED_BUILTIN, LOW);
      // #### ADD ANY SWITCH DEVICE CONTROL HERE ####
      digitalWrite(RELAY_PIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      // #### ADD ANY SWITCH DEVICE CONTROL HERE ####
      digitalWrite(RELAY_PIN, HIGH);
    }
    // Toggle LED to show device is active: Two flashes to OFF if mainsIsOn, to ON if no mains.
    for (int i = 0; i < 2; i++) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  } else {
    // NO COMMS - just alternate LED state at main loop execute frequency.
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
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
  (noCommsCount < NO_COMMS_MAX)  ? noCommsCount++ : noCommsCount ;  // Stop at max. Reset on message received.
  if (noCommsCount == (COMMS_TIMEOUT_SECS * 10) ) {
    haveComms = false;    // At the threshold clear the flag.
  }
  // Delay for a "tick"...
  delay(100);
}
