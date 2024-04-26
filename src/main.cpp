#include <Arduino.h>
#include <lvgl.h>
#include <ATD3.5-S3.h>
#include "gui/ui.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char *ssid = "JD";
const char *password = "Jden4125";
const char *api_url = "https://s6319410013.sautechnology.com/apiproject/check2.php";
const int doorUnlockPin = 13;
const int ledPin = 5;

void connectToWiFi();
void canigetin();
int checkRoomStatus();
void unlockDoor();

void setup()
{
  Serial.begin(115200);
  pinMode(doorUnlockPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  connectToWiFi();

  Display.begin(0);
  Touch.begin();

  Display.useLVGL();
  Touch.useLVGL();
  
  ui_init();

  HTTPClient http;
  http.begin(api_url);
  int httpCode = http.GET();
  if (httpCode > 0)
  {
    String payload = http.getString();
    Serial.println(payload);

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    if (doc.containsKey("roomQRCode")) {
      const char* qrCodeData = doc["roomQRCode"].as<const char*>();
      lv_obj_t *qrcode = lv_qrcode_create(ui_qrcode_body, 180, lv_color_hex(0x000000), lv_color_hex(0xFFFFFF));
      lv_qrcode_update(qrcode, qrCodeData, strlen(qrCodeData));
      lv_obj_center(qrcode);
    }

    if (doc.containsKey("cname")) {
      String cname = doc["cname"].as<String>();
      lv_label_set_text_fmt(ui_cname, "%s", cname.c_str());
    }
    
  }
  else
  {
    Serial.print("Error on HTTP request: ");
    Serial.println(httpCode);
  }

  http.end();

  lv_obj_add_event_cb(ui_btncheck,[](lv_event_t*e){
    canigetin();
  },LV_EVENT_CLICKED,NULL);

}

void canigetin(){

  int roomQRStatus = checkRoomStatus();
  if (roomQRStatus == 0)
  {
    unlockDoor();
  }
  else
  {
    digitalWrite(ledPin, LOW); // ปิด LED ถ้าห้องไม่พร้อมให้ปลดล็อค
    Serial.println("Room is not ready for unlocking");
  }

}

int checkRoomStatus()
{
  HTTPClient http;
  http.begin(api_url);

  int roomQRStatus = -1;

  int httpCode = http.GET();
  if (httpCode > 0)
  {
    String payload = http.getString();
    Serial.println(payload);

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);
    roomQRStatus = doc["roomQRStatus"].as<int>();
    Serial.print("Room QR Status: ");
    Serial.println(roomQRStatus);

    if (roomQRStatus == 0)
    {
      digitalWrite(ledPin, HIGH); // เปิด LED ถ้าห้องพร้อมให้ปลดล็อค
    }
    else
    {
      digitalWrite(ledPin, LOW); // ปิด LED ถ้าห้องไม่พร้อมให้ปลดล็อค
    }
  }
  else
  {
    Serial.print("Error on HTTP request: ");
    Serial.println(httpCode);
  }

  http.end();
  return roomQRStatus;
}

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void unlockDoor()
{
  digitalWrite(doorUnlockPin, HIGH);
  Serial.println("Door unlocked!");
}

void loop()
{
  Display.loop();

}
