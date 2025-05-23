#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// --- UART cho ESP8266 ---
constexpr uint8_t ESP_RX_PIN = 10;
constexpr uint8_t ESP_TX_PIN = 11;
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// --- Kích thước gói dữ liệu 2 chiều ---
constexpr size_t PACKET_SIZE = sizeof(float) * 6;

#define GAS_PIN   A0
#define LDR_PIN   A1
#define DHTPIN    2
#define DHTTYPE   DHT11
#define PIR_PIN   3

const int TRIG_PIN  = 8;
const int ECHO_PIN  = 7;
const int RED_PIN   = 6;
const int GREEN_PIN = 5;
const int BLUE_PIN  = 4;
// --- Servo ---
constexpr uint8_t SERVO_PIN = 9;

// --- Button & Buzzer ---
const int BUTTON_PIN = 12;
const int BUZZER_PIN = 13;

// DHT & OLED setup
DHT dht(DHTPIN, DHTTYPE);

Servo myServo;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float tx_data[6];
float rx_data[6];

unsigned long lastMillis = 0;
bool buzzing = false;
unsigned long buzzStart = 0;
int buzzCount = 0;

// Thông báo khách đến
bool showMessage = false;
unsigned long messageStart = 0;

void setup() {
  // Serial & ESP
  Serial.begin(9600);
  espSerial.begin(9600);
  dht.begin();

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ;
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextWrap(false);

  // Pin modes
  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(0); // Đặt góc ban đầu cho servo

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // nút nhấn nối xuống GND
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  unsigned long now = millis();

  // Nếu đang hiển thị thông báo khách đến
  if (showMessage) {
    // Hiển thị "Có khách đến !!!" suốt 5s
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.println(F(" Co khach"));
    display.setCursor(0, 40);
    display.println(F("  den !!!"));
    display.display();

    // Kết thúc sau 5s
    if (now - messageStart >= 5000) {
      showMessage = false;
      lastMillis = now; // reset timer cho sensor
    }
    // Đồng thời xử lý buzzing
    if (buzzing) {
      unsigned long elapsed = now - buzzStart;
      if (elapsed < 5000) {
        int cycle = elapsed / 1000;
        if (cycle > buzzCount) {
          buzzCount = cycle;
          digitalWrite(BUZZER_PIN, HIGH);
          delay(150);
          digitalWrite(BUZZER_PIN, LOW);
        }
      } else {
        buzzing = false;
        digitalWrite(BUZZER_PIN, LOW);
      }
    }
    return;
  }

  // --- Phần đọc cảm biến, gửi ESP, hiển thị OLED mỗi 1s ---
  if (now - lastMillis >= 1000) {
    lastMillis = now;

    // Đọc cảm biến
    tx_data[0] = dht.readTemperature();    // °C
    tx_data[1] = dht.readHumidity();       // %
    int rawLight = analogRead(LDR_PIN);
    int rawGas   = analogRead(GAS_PIN);
    tx_data[2] = map(rawLight, 0, 1023, 0, 100);
    tx_data[3] = map(rawGas,   0, 1023, 0, 100);
    tx_data[4] = digitalRead(PIR_PIN);
    // Khoảng cách HC-SR04
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long duration = pulseIn(ECHO_PIN, HIGH);
    tx_data[5] = duration / 2.0f / 29.412f;

    // Điều khiển servo
    int servoAngle = 0;
    if (!isnan(tx_data[5]) && tx_data[5] < 5.0f) {
      servoAngle = 90;
    } else {
      servoAngle = 0;
    }
    myServo.write(servoAngle);
    // Gửi dữ liệu lên ESP qua SoftwareSerial
    {
      uint8_t buf[PACKET_SIZE];
      memcpy(buf, tx_data, PACKET_SIZE);
      espSerial.write(buf, PACKET_SIZE);
    }

    // Nhận phản hồi điều khiển LED
    if (espSerial.available() >= PACKET_SIZE) {
      uint8_t buf[PACKET_SIZE];
      espSerial.readBytes(buf, PACKET_SIZE);
      memcpy(rx_data, buf, PACKET_SIZE);
    }

    // Điều khiển LED RGB
    digitalWrite(RED_PIN,   rx_data[0] > 0.5f ? HIGH : LOW);
    digitalWrite(GREEN_PIN, rx_data[1] > 0.5f ? HIGH : LOW);
    digitalWrite(BLUE_PIN,  rx_data[2] > 0.5f ? HIGH : LOW);

    // Hiển thị giá trị cảm biến lên OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("Nhiet do: ")); display.print(tx_data[0],1); display.println(F(" oC"));
    display.setCursor(0, 16);
    display.print(F("Do am:   ")); display.print(tx_data[1],1); display.println(F(" %"));
    display.setCursor(0, 32);
    display.print(F("Anh sang: ")); display.print(tx_data[2],0); display.println(F(" %"));
    display.setCursor(0, 48);
    display.print(F("Khi gas:  ")); display.print(tx_data[3],0); display.println(F(" %"));
    display.display();

    // Debug Serial
    Serial.print("T:"); Serial.print(tx_data[0],1);
    Serial.print(" H:"); Serial.print(tx_data[1],1);
    Serial.print("% L:"); Serial.print(tx_data[2],0);
    Serial.print("% G:"); Serial.print(tx_data[3],0);
    Serial.print("% M:"); Serial.print((int)tx_data[4]);
    Serial.print(" D:"); Serial.println(tx_data[5],1);
  }

  // --- Phát hiện nút nhấn và khởi tạo thông báo + buzzer ---
  if (digitalRead(BUTTON_PIN) == LOW) {
    showMessage = true;
    messageStart = now;
    buzzing = true;
    buzzStart = now;
    buzzCount = 0;
  }

  // Nếu không trong showMessage, buzzer chỉ được điều khiển bên trên
}
  