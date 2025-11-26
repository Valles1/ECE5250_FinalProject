#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

#define STM32_ADDR 0x42   //I2C address of STM32

//define variable for the values we retrieve from the STM32
volatile uint16_t front_distance = 0;
volatile uint16_t rear_distance  = 0;
volatile int16_t  accel_x        = 0;
volatile int16_t  accel_y        = 0;
volatile int16_t  accel_z        = 0;
volatile uint16_t accel_mag      = 0;
volatile uint8_t  Last_Cmd        = 'S';  // default = Stop

//Function used for getting distace and data values from the STM32
void Receive_Event(int Sent_Bytes)
{
  //Three bytes are sent for the ID to distinguish what is sent
  //and to take two bytes for the value of the data sent
  if (Sent_Bytes == 3) {
    uint8_t ID   = Wire.read();
    uint8_t High = Wire.read();
    uint8_t Low  = Wire.read();

    uint16_t value = ((uint16_t)High << 8) | Low;

    switch (ID) {
      case 0x01:  //front distance
        front_distance = value;
        break;

      case 0x02:  //rear distance
        rear_distance = value;
        break;

      case 0x03:  //accel X
        accel_x = (int16_t)value;
        break;

      case 0x04:  //accel Y
        accel_y = (int16_t)value;
        break;

      case 0x05:  //accel Z
        accel_z = (int16_t)value;
        break;

      case 0x06:  //accel magnitude
        accel_mag = value;
        break;

      default:
        // Unknown ID
        break;
    }
  } else {
    
    while (Wire.available()) Wire.read();
  }
}


// This is called whenever the STM32 wants to receive data
void Request_Event()
{
  // Send the latest command as a single byte
  Wire.write(Last_Cmd);
}

//WiFi Name and Password
const char* ssid     = "ESP32-RC-Car";
const char* password = "12345678";

WebServer server(80);

//Code for building the webpage used to control motor and see live data
String Build_WebPage() {
  String html = 
    "<html>"
    "<head>"
    "<style>"
      "body { font-family: Arial; text-align:center; }"
      "h1 { color:#003366; }"
      "button { width:120px; height:50px; font-size:20px; margin:10px; }"
      "div { font-size:24px; margin-top:20px; }"
    "</style>"

    "<script>"
    "function sendCmd(cmd) {"
       "fetch('/cmd?val=' + cmd);"
    "}"

    "function updateDistances() {"
        "fetch('/dist').then(response => response.json()).then(data => {"
            "document.getElementById('front').innerHTML = data.front;"
            "document.getElementById('rear').innerHTML  = data.rear;"
            "document.getElementById('accel').innerHTML = "
              "'X=' + data.ax + ', Y=' + data.ay + ', Z=' + data.az + ', |Mag|=' + data.amag;"
        "});"
    "}"
    "setInterval(updateDistances, 250);"    // update every 250 ms
    "</script>"
    "</head>"

    "<body>"
    "<h1>RC Car Controller</h1>"

    "<div>"
      "<button onclick=\"sendCmd('F')\">Forward</button>"
    "</div>"
    "<div>"
      "<button onclick=\"sendCmd('L')\">Left</button>"
      "<button onclick=\"sendCmd('S')\">Stop</button>"
      "<button onclick=\"sendCmd('R')\">Right</button>"
    "</div>"
    "<div>"
      "<button onclick=\"sendCmd('B')\">Reverse</button>"
    "</div>"

    "<div>Front Distance: <span id='front'>--</span> cm</div>"
    "<div>Rear Distance: <span id='rear'>--</span> cm</div>"
    "<div>Accel (g): <span id='accel'>X=--, Y=--, Z=--, |Mag|=--</span></div>"

    "</body>"
    "</html>";

  return html;
}


// Send webpage
void Handle_Web_Root() {
  server.send(200, "text/html", Build_WebPage());
}

//sends values for distances
void Handle_Distance_Vals() {
  String json = "{";
  json += "\"front\":" + String(front_distance) + ",";
  json += "\"rear\":"  + String(rear_distance)  + ",";
  json += "\"ax\":"    + String(accel_x)        + ",";
  json += "\"ay\":"    + String(accel_y)        + ",";
  json += "\"az\":"    + String(accel_z)        + ",";
  json += "\"amag\":"  + String(accel_mag);
  json += "}";
  server.send(200, "application/json", json);
}

// Receive command from browser and update Last_Cmd
void Handle_Command() {
  if (!server.hasArg("val")) {
    server.send(400, "text/plain", "Missing val param");
    return;
  }

  String cmd = server.arg("val");

  if (cmd.length() > 0) {
    Last_Cmd = (uint8_t)cmd[0];  // store as single char for STM32
  }

  Serial.print("Received CMD: ");
  Serial.println(cmd);

  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Starting I2C + WiFi AP...");

  // ESP32 acts as I2C receiver at address 0x42
  Wire.begin(STM32_ADDR);
  Wire.onReceive(Receive_Event);   // STM32 writes the distances + accel
  Wire.onRequest(Request_Event);   // STM32 reads Last_CMd

  //for creating the WiFi AP
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  //for updating the webpage depending on commands and live data
  server.on("/", Handle_Web_Root);
  server.on("/dist", Handle_Distance_Vals);
  server.on("/cmd", Handle_Command);
  server.begin();

  Serial.println("Web server started!");
}

void loop() {
  server.handleClient();
  delay(1);
}