#include <esp_now.h>
#include <WiFi.h>

String data;

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void OnDataRecv(const esp_now_recv_info* recvInfo, const uint8_t *incomingData, int len){
  memcpy(&data, incomingData, sizeof(data));
 // Serial.print("Data received: ");
 // Serial.println(len);
 Serial.println(data);
 Serial.println();
 handle(data);
}

void handle(String data)
{
  if (data.startsWith("#A"))
  {
    String numberPart = data.substring(2); // Skip '#' and 'A'
    Serial.println("Number part: " + numberPart);
  }
}