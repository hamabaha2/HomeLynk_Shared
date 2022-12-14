// Comments with ** require work

#include <WiFi.h>

char home_ssid[]     = "SSID_here";
char home_password[] = "password_here";

void setup()
{
    // Connecting to a WiFi network

    // indicators for attempting to connecting to Wifi here **
    
    // Begin connecting 
    
    WiFi.begin(home_ssid, home_password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        
        // Create exit loop for wifi timeout here**
    }
    
    if (WiFi.status() != WL_CONNECTED) {
    
    // Indicate WiFi connection failed and end this routine **
    
    }
    
    
    
}
 // below are examples on how to connect to a server

 // Use WiFiClient class to create TCP connections
    WiFiClient client;
    const int httpPort = 80;
    if (!client.connect(host, httpPort)) {
        Serial.println("connection failed");
        return;

// This will send the request to the server

    char url = “url here”
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");
    unsigned long timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println(">>> Client Timeout !");
            client.stop();
            return;
