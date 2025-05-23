#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// --- WiFi & MQTT cấu hình ---
const char* ssid        = "CONG";
const char* password    = "11111111";
const char* mqtt_broker = "broker.emqx.io";
const char* mqtt_user   = "IOT_Cong";
const char* mqtt_pass   = "12042005";
const int   mqtt_port   = 1883;

// --- Buffer float ---
float rx_data[6] = {0};
float tx_data[2] = {1.0f, 0.0f};

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastPub = 0;
const unsigned long interval = 1000;

void ensureWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(200);
    }
  }
}

void setup() {
  // Serial: debug console + giao tiếp với UNO
  Serial.begin(74880);

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  // Kết nối MQTT
  mqttClient.setServer(mqtt_broker, mqtt_port);
  String clientId = "esp8266-" + String(ESP.getChipId());
  while (!mqttClient.connected()) {
    mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass);
    delay(500);
  }
}

void loop() {
  ensureWifi();
  mqttClient.loop();

  // 1) Đọc 6 float từ UNO qua UART0 (Serial)
  constexpr size_t BUF_IN = sizeof(float) * 6;
  if (static_cast<size_t>(Serial.available()) >= BUF_IN) {
    uint8_t buf[BUF_IN];
    Serial.readBytes(buf, BUF_IN);
    memcpy(rx_data, buf, BUF_IN);
  }

  // 2) Publish mỗi 1 giây
  unsigned long now = millis();
  if (now - lastPub >= interval) {
    lastPub = now;
    float t     = rx_data[0];
    float h     = rx_data[1];
    float light = rx_data[3];
    float gas   = rx_data[4];
    float mot   = rx_data[5];

    mqttClient.publish("sensor/temperature", String(t,2).c_str());
    mqttClient.publish("sensor/humidity",    String(h,2).c_str());
    mqttClient.publish("sensor/light",       String(light,2).c_str());
    mqttClient.publish("sensor/gas",         String(gas,2).c_str());
    mqttClient.publish("sensor/motion",      String(mot,0).c_str());

    // Debug console
    Serial.printf("Pub T=%.2f H=%.2f L=%.2f G=%.2f M=%.0f\n",
                  t, h, light, gas, mot);

    // 3) Gửi lại UNO 2 float status qua UART0
    constexpr size_t BUF_OUT = sizeof(float) * 2;
    uint8_t outb[BUF_OUT];
    memcpy(outb, tx_data, BUF_OUT);
    Serial.write(outb, BUF_OUT);
  }
}
