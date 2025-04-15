#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "Airtel_vish_0787";
const char* password = "air76152";

// Define static IP configuration
IPAddress local_IP(192, 168, 1, 200);  // Set a static IP address
IPAddress gateway(192, 168, 1, 1);      // Set your router's gateway
IPAddress subnet(255, 255, 255, 0);     // Set subnet mask
IPAddress primaryDNS(8, 8, 8, 8);       // (Optional) Set a primary DNS
IPAddress secondaryDNS(8, 8, 4, 4);     // (Optional) Set a secondary DNS

ESP8266WebServer server(80);

void setup() {
    Serial.begin(9600);  
    delay(1000);  

    // Connect to WiFi with a static IP
    WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to WiFi");
    Serial.print("ESP8266 IP Address: ");
    Serial.println(WiFi.localIP());  // Print the assigned static IP

    server.on("/command", HTTP_GET, []() {
        if (server.hasArg("cmd")) {
            String command = server.arg("cmd");
            command.trim();  

            Serial.println(command);  // Send only the command to Arduino

            server.send(200, "text/plain", "Command Sent: " + command);
        } else {
            server.send(400, "text/plain", "Missing 'cmd' parameter");
        }
    });

    server.begin();
}

void loop() {
    server.handleClient();
}
