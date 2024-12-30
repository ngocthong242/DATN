#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <DHT.h>  // Thư viện cho DHT11
#include <LiquidCrystal_I2C.h>  // Thư viện cho LCD
#include <ESP32Servo.h>  // Thư viện cho Servo

#define SDA_PIN 21  // Chân SDA cho ESP32
#define SCL_PIN 22  // Chân SCL cho ESP32

// Định nghĩa chân DHT11
#define DHTPIN 23         // Pin DHT11
#define DHTTYPE DHT11     // DHT11
DHT dht(DHTPIN, DHTTYPE);

// Định nghĩa các chân cảm biến và relay
#define soilMoisturePin 36        // Pin cảm biến độ ẩm đất
#define rainSensorPin 4           // Pin cảm biến mưa
#define pumpRelayPin 19           // Pin Relay bơm
#define lightRelayPin 18          // Pin Relay đèn
#define mistRelayPin 17           // Pin Relay sương
#define servoPin 27              // Pin điều khiển Servo

// Web server object
WebServer server(80);

// Sensor objects
BH1750 lightMeter(0x23);  // Địa chỉ mặc định của BH1750

// LCD setup
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Servo setup
Servo myservo; // Tạo đối tượng Servo sử dụng ESP32Servo

const char* ssid = "MyPhone";  // Thay đổi với tên Wi-Fi của bạn
const char* password = "1234567890";  // Thay đổi với mật khẩu Wi-Fi của bạn

void manhinh(int lux, float temperature, float humidityAir, int humiditySoil, int rainState, int pumpState, int lightState, int mistState, int roofState) {
  // Cập nhật nhãn trên màn hình LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L:"); // Light
  lcd.setCursor(8, 0);
  lcd.print("lx");
  lcd.setCursor(11, 0);
  lcd.print("S:"); // Soil moisture
  lcd.setCursor(19, 0);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("T:"); // Temperature
  lcd.setCursor(8, 1);
  lcd.print("*C");
  lcd.setCursor(11, 1);
  lcd.print("H:"); // Humidity
  lcd.setCursor(19, 1);
  lcd.print("%");

  lcd.setCursor(0, 2);
  lcd.print("PUMP:");
  lcd.setCursor(11, 2);
  lcd.print("LED:");
  lcd.setCursor(0, 3);
  lcd.print("MIST:");
  lcd.setCursor(11, 3);
  lcd.print("ROOF:");

  // Hiển thị giá trị cảm biến ánh sáng
  lcd.setCursor(3, 0);
  lcd.print("    "); // Clear previous value
  lcd.setCursor(3, 0);
  lcd.print(lux); // Display new value

  // Hiển thị giá trị cảm biến độ ẩm đất
  lcd.setCursor(13, 0);
  lcd.print("   "); // Clear previous value
  lcd.setCursor(13, 0);
  lcd.print(humiditySoil);

  // Hiển thị giá trị nhiệt độ
  lcd.setCursor(2, 1);
  lcd.print("     "); // Clear previous value
  lcd.setCursor(2, 1);
  lcd.print(temperature);

  // Hiển thị giá trị độ ẩm không khí
  lcd.setCursor(13, 1);
  lcd.print("     "); // Clear previous value
  lcd.setCursor(13, 1);
  lcd.print(humidityAir);

  // Hiển thị trạng thái các relay
  lcd.setCursor(6, 2);
  lcd.print(pumpState ? "ON " : "OFF");
  lcd.setCursor(16, 2);
  lcd.print(lightState ? "ON " : "OFF");
  lcd.setCursor(6, 3);
  lcd.print(mistState ? "ON " : "OFF");
  lcd.setCursor(16, 3);
  lcd.print(roofState ? "ON" : "OFF");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Khởi tạo I2C
  Wire.begin(SDA_PIN, SCL_PIN);  // Sử dụng chân SDA và SCL cụ thể

  // Kết nối Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("\u0110ịa chỉ IP của ESP32: ");
  Serial.println(WiFi.localIP());
  // Khởi tạo cảm biến BH1750
  if (lightMeter.begin()) {
    Serial.println("Cảm biến BH1750 đã sẵn sàng.");
  } else {
    Serial.println("[ERROR] Không tìm thấy cảm biến BH1750.");
    return;
  }

  // Khởi tạo cảm biến DHT11
  dht.begin();

  // Khởi tạo LCD
  lcd.init();
  lcd.backlight();

  // Khởi tạo Servo
  myservo.attach(servoPin);
  myservo.write(90);  // Vị trí ban đầu của servo

  // Đặt trạng thái mặc định cho các relay
  pinMode(pumpRelayPin, OUTPUT);
  pinMode(lightRelayPin, OUTPUT);
  pinMode(mistRelayPin, OUTPUT);
 pinMode(rainSensorPin, INPUT);
  digitalWrite(pumpRelayPin, LOW);
  digitalWrite(lightRelayPin, LOW);
  digitalWrite(mistRelayPin, LOW);

  // Đăng ký HTTP GET route cho "/data"
  server.on("/data", HTTP_GET, []() {
    // Đọc giá trị cảm biến
    float luxRaw = lightMeter.readLightLevel();
    int lux = round(luxRaw); // Convert lux to integer
    float temperature = dht.readTemperature();  // Nhiệt độ từ DHT11
    float humidityAir = dht.readHumidity();    // Độ ẩm không khí từ DHT11

    // Đọc giá trị cảm biến độ ẩm đất
    int humiditySoilRaw = analogRead(soilMoisturePin);  
    int humiditySoil = map(humiditySoilRaw, 0, 4095, 200, 0);  // Độ ẩm đất (100 - 0%)

    // Đọc trạng thái mái (servo)
    int roofState = (myservo.read() > 90) ? 1 : 0;
   // Đọc trạng thái cảm biến mưa
    int rainState = digitalRead(rainSensorPin);

    // Lấy trạng thái các relay
    int pumpState = digitalRead(pumpRelayPin);
    int lightState = digitalRead(lightRelayPin);
    int mistState = digitalRead(mistRelayPin);

    // Tạo đối tượng JSON
    DynamicJsonDocument doc(512);
    doc["LightIntensity"] = lux;
    doc["Temperature"] = round(temperature * 10.0) / 10.0;  // Làm tròn nhiệt độ
    doc["HumidityAir"] = round(humidityAir * 10.0) / 10.0;  // Làm tròn độ ẩm không khí
    doc["HumiditySoil"] = humiditySoil;  // Độ ẩm đất sau khi chuyển đổi sang phần trăm
    doc["Pump"] = pumpState;
    doc["Light"] = lightState;
    doc["Mist"] = mistState;
    doc["Roof"] = roofState;
 doc["Rain"] = rainState;
    // Serialize JSON và gửi trả kết quả
    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
  });

  // Đăng ký HTTP POST route cho "/control"
  server.on("/control", HTTP_POST, []() {
    // Kiểm tra nếu dữ liệu POST có ở dạng JSON
    if (server.hasArg("plain")) {
      String jsonStr = server.arg("plain");
      DynamicJsonDocument doc(200);
      DeserializationError error = deserializeJson(doc, jsonStr);

      // Kiểm tra nếu dữ liệu JSON hợp lệ
      if (error) {
        Serial.println("Invalid JSON received");
        server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
      }

      // In ra dữ liệu JSON nhận được
      serializeJson(doc, Serial);
      Serial.println();

      // Lấy giá trị relay từ JSON
      if (doc.containsKey("Pump")) {
        int pumpState = doc["Pump"];
        digitalWrite(pumpRelayPin, pumpState);
      }
      if (doc.containsKey("Light")) {
        int lightState = doc["Light"];
        digitalWrite(lightRelayPin, lightState);
      }
      if (doc.containsKey("Mist")) {
        int mistState = doc["Mist"];
        digitalWrite(mistRelayPin, mistState);
      }
      if (doc.containsKey("Roof")) {
        static int lastRoofState = -1; // Biến lưu trạng thái Roof trước đó
        int roofState = doc["Roof"];
        if (roofState != lastRoofState) { // Chỉ thay đổi nếu trạng thái khác trước đó
          if (roofState == 0) {
            for (int pos = 130; pos >= 90; pos--) {
              myservo.write(pos);
              delay(15);
            }
          } else if (roofState == 1) {
            for (int pos = 90; pos <= 130; pos++) {
              myservo.write(pos);
              delay(15);
            }
          }
          lastRoofState = roofState;
        }
      }

      // Tạo đối tượng JSON phản hồi với trạng thái relay đã thay đổi
      DynamicJsonDocument responseDoc(200);
      responseDoc["Pump"] = digitalRead(pumpRelayPin);
      responseDoc["Light"] = digitalRead(lightRelayPin);
      responseDoc["Mist"] = digitalRead(mistRelayPin);
      responseDoc["Roof"] = (myservo.read() > 90) ? 1 : 0;

      String responseString;
      serializeJson(responseDoc, responseString);
      server.send(200, "application/json", responseString);
    } else {
      server.send(400, "application/json", "{\"error\":\"No data provided\"}");
    }
  });

  server.begin();
}

void loop() {
  // Cập nhật server
  server.handleClient();

  // Đọc cảm biến và cập nhật LCD
  float luxRaw = lightMeter.readLightLevel();
  int lux = round(luxRaw);
  float temperature = dht.readTemperature();
  float humidityAir = dht.readHumidity();
  int humiditySoilRaw = analogRead(soilMoisturePin);
  int humiditySoil = map(humiditySoilRaw, 0, 4095, 200, 0);

  // Đọc trạng thái mái (servo)
  int roofState = (myservo.read() > 90) ? 1 : 0;
  int rainState = digitalRead(rainSensorPin);
  manhinh(lux, temperature, humidityAir, humiditySoil, rainState, digitalRead(pumpRelayPin), digitalRead(lightRelayPin), digitalRead(mistRelayPin), roofState);

  delay(500);
}
