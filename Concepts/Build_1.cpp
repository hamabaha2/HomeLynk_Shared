# include <WiFi.h>

// Global constants

const int led1_pin = 48;
const int btn_pin = 1;
char* ssid = "MOTOEDA2";
char* passwd = "ka24yxsr4f";

// Set webserver port to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output45State = "off";
String output48State = "off";

// Assign output variables to GPIO pins
const int output45 = 45;
const int output48 = 48;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  // setup serial and pins modes
  
  Serial.begin(115200);
  pinMode(led1_pin, OUTPUT);
  pinMode(btn_pin, INPUT);
  pinMode(output45, OUTPUT);
  pinMode(output48, OUTPUT);
  // Feedback
  Serial.println("Pins modes ... set!");
  // set outputs to low
  digitalWrite(output45, LOW);
  digitalWrite(output48, LOW);
  
  // connect to wifi
  WiFi.begin(ssid,passwd);
  Serial.print("Connecting to wifi ");

  // set Wifi server
  //WiFiServer server(80);


  
  // indicate connection status
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(led1_pin, HIGH);
    delay(400);
    digitalWrite(led1_pin, LOW);
    delay(100);
  }

  Serial.println(" ");
  Serial.println("Wifi Connected! ");
  delay(2000);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  //begin server
  server.begin(80);
  Serial.println("Server begin");
}

void loop() {
  int btn_sts = 0;
  // react when button pressed
  if (digitalRead(btn_pin)==HIGH) { 
    Serial.println("Button was pressed!");
    digitalWrite(output45, HIGH);
    digitalWrite(output48, HIGH);
    delay(200);
    digitalWrite(output45, LOW);
    digitalWrite(output48, LOW);
    delay(300);
    if (output45State!="off") {
      digitalWrite(output45, HIGH);
    }
  if (output48State!="off") {
    digitalWrite(output48, HIGH);
  }
  }


  WiFiClient client = server.available();  //  Listen for incoming clients

  if (client) {                           // if a new client connects
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client");         // Print message that client connected
    String currentLine = "";
    while (client.connected() && (currentTime - previousTime <= timeoutTime)) { // loop while the client cnctd
      currentTime = millis();
      if (client.available()) {         // if there is a byte to read from the client,
        char c = client.read();         // read a byte, then
        Serial.write(c);                // printing it in serial
        header += c;
        if (c == '\n') {
          // if the line is blankc, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() ==0) {
            // HTTP header always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the led's on and off
            if (header.indexOf("GET /45/on") >= 0) {
              Serial.println("led 45 on");
              output45State = "on";
              digitalWrite(output45, HIGH);
            } else if (header.indexOf("GET /45/off") >= 0) {
              Serial.println("led 45 off");
              output45State = "off";
              digitalWrite(output45, LOW);
            } else if (header.indexOf("GET /48/on") >= 0) {
              Serial.println("led 48 on");
              output48State = "on";
              digitalWrite(output48, HIGH);
            } else if (header.indexOf("GET /48/off") >= 0) {
              Serial.println("led 48 off");
              output48State = "off";
              digitalWrite(output48, LOW);
            }

            // Display the HTML webpage
                        client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");

            // Display current state, and ON/OFF buttons for GPIO 45
            client.println("<p>GPIO 45 - State " + output45State + "</p>");
            // If the output45State is off, it displays the ON button
            if (output45State == "off") {
              client.println("<p><a href=\"/45/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/45/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            
            // Display current state, and ON/OFF buttons for GPIO 48
            client.println("<p>GPIO 48 - State " + output48State + "</p>");
            // If the output48State is off, it displays the ON button
            if (output48State == "off") {
              client.println("<p><a href=\"/48/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/48/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Adding a form and a submit ******

            
            client.println("<form class=\"\" action=\"index.html\" method=\"post\">");
            client.println("<label for=\"\">Enter Command:</label>");
            client.println("<input type=\"text\" name=\"cmd\" value=\"/26/on\">");
            client.println("<input type=\"submit\" name=\"cmd\">");
            client.println("</form>");



            // end of form and submit *******





            
            client.println("</body></html>");

            //The HTTP response ends with another blank line
            client.println();
            // Break our of the whilte look
            break;
          } else { // if you got a newline, then clear current line
            currentLine = "";
          }
       } else if (c != '\r') { // if you got anything else but a carriage return character,
          currentLine += c;    // add it to the end of the currenLine
       }
    }
  }
  // clear the header variable
  header = "";
  // close the connection
  client.stop();
  Serial.println("Client disconnected.");
  Serial.println("");
}
}

