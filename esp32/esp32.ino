//_______ ESP32 ____________ //

#include <WiFi.h>
#include <HTTPClient.h>
#include <ThingsBoard.h>


const char token[] = "d07tm6sttgs1wwxr50yc";
const char server[] = "http://thingsboard.cloud/api/v1/d07tm6sttgs1wwxr50yc/telemetry";
const char webhook_key[] = "dGjFO-aRX17P4DWvo9060-";
const char ifttt_trigger[] = "http://maker.ifttt.com/trigger/event/with/key/dGjFO-aRX17P4DWvo9060-";

#define RXD0 3
#define TXD0 1

const char* ssid = "hello";
const char* password = "daH0tspotpass&";


WiFiClient espClient;
HTTPClient http;
int status = WL_IDLE_STATUS;

void setup(){
  Serial.begin(115200);
  // Serial.begin(115200, SERIAL_8N1,RXD0,TXD0); // code workes even without defining tx and rx pin*

  delay(1000);
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");
    while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
    }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void loop(){
  Serial.print(Serial.readString());
  delay(500);

  http.begin(espClient, server);
  http.addHeader("Content-Type", "application/json");
  String httpRequestData = Serial.readString();
  int httpResponseCode = http.POST(httpRequestData);

  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String httpReqData = ifttt_trigger;
  int httpResCode = http.POST(httpReqData);

}