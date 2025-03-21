#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

Servo myServo;
QueueHandle_t stateFanQueue;
QueueHandle_t statePumpQueue;
QueueHandle_t stateServorQueue;
QueueHandle_t temperaturaQueue;
QueueHandle_t humidityQueue;
QueueHandle_t airQueue;


#define OLED_ADDR   0x3C  
#define DHTTYPE     DHT11 

#define DHTPIN      4     // Cảm biến DHT11
#define AIR_SENSOR  35    // Cảm biến khí (ADC)

#define CUSTOM_SDA  21    // I2C OLED SDA
#define CUSTOM_SCK  22    // I2C OLED SCL

#define OLED_BUTTON_PIN  15  // Nút nhấn OLED

// 🔹 **Chân siêu âm**
#define TRIG_PIN1        18
#define ECHO_PIN1        19

// 🔹 **Nút nhấn**
#define PUMP_BUTTON_PIN  12  // Nút nhấn bơm
#define FAN_BUTTON_PIN   14  // Nút nhấn quạt
#define SERVO_BUTTON_PIN 13  // Nút nhấn servo

// 🔹 **Điều khiển relay**
#define PUMP_RELAY_PIN   27  // Relay bơm
#define FAN_RELAY_PIN    26  // Relay quạt
#define SERVO_PIN        17  // Servo PWM


WiFiClient espClient;
PubSubClient client(espClient);

// Wi-Fi and MQTT settings
const char *ssid PROGMEM = "POCO F5 Pro";
const char *password PROGMEM = "55555555";
const char *mqtt_broker PROGMEM = "broker.emqx.io";
const char *topic0 PROGMEM = "esp32/tem";
const char *topic1 PROGMEM = "esp32/hum";
const char *topic2 PROGMEM = "esp32/air";
const char *topic3 PROGMEM = "esp32/foodrate";
const char *topic4 PROGMEM = "esp32/quat/control";
const char *topic5 PROGMEM = "esp32/maybom/control";
const char *topic6 PROGMEM = "esp32/fan/mode";
const char *topic7 PROGMEM = "esp32/servo/control";
const char *topic8 PROGMEM = "esp32/den/control";
const char *topic9 PROGMEM = "esp32/maybom2/control";
const char *topic10 PROGMEM = "esp32/quat/test";
const char *topic11 PROGMEM = "esp32/maybom1/test";
const char *topic12 PROGMEM = "esp32/maybom2/test";
const char *topic13 PROGMEM = "esp32//test";
const char *topic14 PROGMEM = "esp32/waterrate";
const char *topic15 PROGMEM = "targetHour";
const char *topic16 PROGMEM = "targetMinute";
const char *topic17 PROGMEM = "targetSecond";
const char *mqtt_username PROGMEM = "HaiDuong";
const char *mqtt_password PROGMEM = "123456";
const int mqtt_port PROGMEM = 1883;



// 🔹 **Khai báo nguyên mẫu hàm**
void readUltrasonicSensor(void *pvParameters);
void handlePumpControl(void *pvParameters);
void handleServoControl(void *pvParameters);
void handleFanControl(void *pvParameters);
void mqttLoopTask(void *pvParameters);
void callback(char *topic, byte *payload, unsigned int length);
void vReceiverSensor(void *pvParameters);
void vSenderDHTTemperature(void *pvParameters);
void vSenderDHTHumidity(void *pvParameters);
void vSenderAIR(void *pvParameters);

Adafruit_SSD1306 display(128, 64, &Wire, -1);
DHT dht(DHTPIN, DHTTYPE);



void setup() {
    Serial.begin(9600);
    myServo.attach(SERVO_PIN);
    myServo.write(90);  // Gán vị trí ban đầu cho Servo
    Wire.begin();
    dht.begin();
    pinMode(AIR_SENSOR, INPUT); 
    


    // 🔹 **Cấu hình chân I/O**
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT_PULLDOWN);

    pinMode(PUMP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FAN_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SERVO_BUTTON_PIN, INPUT_PULLUP);

    pinMode(PUMP_RELAY_PIN, OUTPUT);
    pinMode(FAN_RELAY_PIN, OUTPUT);
    pinMode(OLED_BUTTON_PIN, INPUT_PULLUP);

    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FAN_RELAY_PIN, LOW);

    // Create Queue
    
    stateFanQueue = xQueueCreate(5, sizeof(char[4])); 
    statePumpQueue = xQueueCreate(5, sizeof(char[4]));
    stateServorQueue = xQueueCreate(5, sizeof(char[4]));
    temperaturaQueue = xQueueCreate(1, sizeof(float));
    humidityQueue = xQueueCreate(1, sizeof(float));
    airQueue = xQueueCreate(1, sizeof(float));

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(20, 20);
    display.println("Starting....");
    display.display();



   // Kết nối WiFi
   WiFi.begin(ssid, password);
   Serial.print("Connecting to WiFi...");
   while (WiFi.status() != WL_CONNECTED) {
       delay(500);
       Serial.print(".");
   }
   Serial.println("\nConnected to WiFi!");

   // Cấu hình MQTT
   client.setServer(mqtt_broker, mqtt_port);
   client.setCallback(callback);

   // Kết nối MQTT với retry
   int retryCount = 0;
   while (!client.connected() && retryCount < 5) {  
       Serial.println("Attempting MQTT connection...");
       String client_id = "esp32-client-" + String(WiFi.macAddress());
       if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
           Serial.println("Connected to MQTT broker");
       } else {
           Serial.print("Failed, rc=");
           Serial.print(client.state());  // In mã lỗi kết nối
           Serial.println(". Trying again in 2 seconds...");
           delay(2000);
       }
       retryCount++;
   }
   if (!client.connected()) {
       Serial.println("MQTT connection failed. Restarting...");
       ESP.restart();  // Khởi động lại nếu không kết nối được
   }

  client.subscribe(topic4);
  client.subscribe(topic5);
  client.subscribe(topic6);
  client.subscribe(topic7);
  client.subscribe(topic8);
  client.subscribe(topic9);
  client.subscribe(topic15);
  client.subscribe(topic16);
  client.subscribe(topic17);


    // 🔹 **Tạo các task FreeRTOS**
    xTaskCreate(readUltrasonicSensor, "readUltrasonicSensor", 8192, NULL, 2, NULL);
    xTaskCreate(handlePumpControl, "handlePumpControl", 8192, NULL, 4, NULL);
    xTaskCreate(handleServoControl, "handleServoControl", 8192, NULL, 4, NULL);
    xTaskCreate(handleFanControl, "handleFanControl", 8192, NULL, 4, NULL);
    xTaskCreate( mqttLoopTask, "mqttLoopTask", 8192, NULL, 5, NULL);

    xTaskCreate(vSenderDHTTemperature, "SenderTemp", 2048, NULL, 1, NULL);
    xTaskCreate(vSenderDHTHumidity, "SenderHumi", 2048, NULL, 1, NULL);
    xTaskCreate(vSenderAIR, "SenderAir", 2048, NULL, 1, NULL);
    xTaskCreate(vReceiverSensor, "Receiver", 4096, NULL, 1, NULL);
    
}

void loop() {
    // Không làm gì trong loop vì dùng FreeRTOS
}

void mqttLoopTask(void *pvParameters) {
    for (;;) {
        client.loop();
        vTaskDelay(pdMS_TO_TICKS(10));  // Tránh chiếm CPU quá mức
    }
}


// 🔹 **Task đọc cảm biến siêu âm**
void readUltrasonicSensor(void *pvParameters) {
    long duration1;
    float distanceOrigin = 8.0;

    for (;;) {
        digitalWrite(TRIG_PIN1, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN1, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN1, LOW);

        duration1 = pulseIn(ECHO_PIN1, HIGH, 30000); // Timeout 30ms
        float distance = (duration1 / 2.0) / 29.412;

        Serial.print("Distance: ");
        Serial.println(distance);

        float foodAvailable = (1.0 - (distance / distanceOrigin)) * 100.0;
        Serial.print("Food rate: ");
        Serial.println(foodAvailable);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// 🔹 **Task điều khiển máy bơm**
void handlePumpControl(void *pvParameters) {
    bool isPumpOn = false;
    bool lastPumpButtonState = HIGH;
    char statePumpQueueValue[4];
    for (;;) {
        bool pumpButtonState = digitalRead(PUMP_BUTTON_PIN);

        if (pumpButtonState == LOW && lastPumpButtonState == HIGH) {
            isPumpOn = !isPumpOn;
            digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
            client.publish(topic5, isPumpOn ? "ON" : "OFF"); 
            strcpy(statePumpQueueValue, isPumpOn ? "ON" : "OFF"); 
            xQueueSend(statePumpQueue, &statePumpQueueValue, 0);
        }
       
        
        
        lastPumpButtonState = pumpButtonState;
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Debounce nút nhấn
    }
}

// 🔹 **Task điều khiển Servo**
void handleServoControl(void *pvParameters) {
    bool isServoAt90 = false;
    bool lastServoButtonState = HIGH;
    

    for (;;) {
        bool currentServoButtonState = digitalRead(SERVO_BUTTON_PIN);
        char stateServoQueueValue[4];

        if (currentServoButtonState == LOW && lastServoButtonState == HIGH) {
            isServoAt90 = !isServoAt90;
            myServo.write(isServoAt90 ? 90 : 0);
            strcpy(stateServoQueueValue, isServoAt90 ? "ON" : "OFF"); 
            xQueueSend(stateServorQueue, &stateServoQueueValue, 0);
            client.publish(topic7, isServoAt90 ? "ON" : "OFF");
        }
        
        lastServoButtonState = currentServoButtonState;
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Debounce
    }
}
 
// 🔹 **Task điều khiển quạt**
void handleFanControl(void *pvParameters) {
    bool isFanOn = false;
    bool lastFanButtonState = HIGH;

    for (;;) {
        bool fanButtonState = digitalRead(FAN_BUTTON_PIN);
        char stateFanQueueValue[4];
        if (fanButtonState == LOW && lastFanButtonState == HIGH) {
            isFanOn = !isFanOn;
            client.publish(topic4, isFanOn ? "ON" : "OFF");
            digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
            strcpy(stateFanQueueValue, isFanOn ? "ON" : "OFF"); 
            xQueueSend(stateFanQueue, &stateFanQueueValue, 0);
        }

        lastFanButtonState = fanButtonState;
        vTaskDelay(pdMS_TO_TICKS(50));  // Debounce
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    String message;
    bool isPumpOn = false;
    bool isFanOn = false;
    bool isServoAt90 = false;
    char statePumpQueueValue[4];
    char stateServoQueueValue[4];
    char stateFanQueueValue[4];
    
    for (int i = 0; i < length; i++) message += (char)payload[i];
  
    if (String(topic) == topic4) isFanOn = (message == "ON");
    if (String(topic) == topic5) isPumpOn = (message == "ON");
    //  if (String(topic) == topic6) currentMode = (message == "MANUAL") ? MANUAL : AUTOMATIC;
    if (String(topic) == topic7) {
        isServoAt90 = (message == "ON");
        myServo.write(isServoAt90 ? 90 : 0);
    }
    // if (String(topic) == topic8) isBulbOn = (message == "ON");
    // if (String(topic) == topic9) isPump2On = (message == "ON");
    // if (String(topic) == topic15) targetHour = message.toInt() ;
    // if (String(topic) == topic16) targetMinute = message.toInt() ;
    // if (String(topic) == topic17) 
    // {targetSecond = message.toInt() ;
    //  targetSecondclose = targetSecond +5;}
    
    digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
    digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
    // digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);
    // digitalWrite(BULB_RELAY_PIN, isBulbOn ? HIGH : LOW);
    strcpy(statePumpQueueValue, isPumpOn ? "ON" : "OFF"); 
        xQueueSend(statePumpQueue, &statePumpQueueValue, 0);
    strcpy(stateServoQueueValue, isServoAt90 ? "ON" : "OFF"); 
        xQueueSend(stateServorQueue, &stateServoQueueValue, 0);
    strcpy(stateFanQueueValue, isFanOn ? "ON" : "OFF"); 
        xQueueSend(stateFanQueue, &stateFanQueueValue, 0);     
  }

  
void vSenderDHTTemperature(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        float temperatureValue = dht.readTemperature();
        if (!isnan(temperatureValue) && temperatureValue > 0) {
            xQueueSend(temperaturaQueue, &temperatureValue, portMAX_DELAY);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void vSenderDHTHumidity(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        float humidityValue = dht.readHumidity();
        if (!isnan(humidityValue) && humidityValue > 0) {
            xQueueSend(humidityQueue, &humidityValue, portMAX_DELAY);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void vSenderAIR(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        float airValue = analogRead(AIR_SENSOR);
        if (airValue > 0) {
            xQueueSend(airQueue, &airValue, portMAX_DELAY);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}



void vReceiverSensor(void *pvParameters) {
  float temp = 0, humi = 0, air = 0;
  bool lastOledButtonState = HIGH;
  bool isSwitch = false;
  int currentScreen = 0;

  for (;;) {
    int OledButtonState = digitalRead(OLED_BUTTON_PIN);
    bool updated = false;
    char statePump[4];
    
    // Kiểm tra trạng thái nút bấm OLED
    if (OledButtonState == LOW && lastOledButtonState == HIGH) {
      currentScreen = (currentScreen + 1) % 3;  // Chuyển màn hình
     // delay(200);  // Chống hiện tượng nhấn nút nhiều lần do phản hồi
    }
    lastOledButtonState = OledButtonState;
    
    // Dựa trên màn hình hiện tại để hiển thị dữ liệu
    switch (currentScreen) {
      case 0: 
      if ( xQueueReceive(statePumpQueue, &statePump,0)==pdPASS)
          {
            Serial.println( statePump);
          }
        display.clearDisplay();  // Xóa màn hình
        display.setCursor(20, 20);
        display.print(statePump);    
        display.display();
        break;

      case 1:
        updated = false;

        // Nhận dữ liệu từ các queue và hiển thị trên màn hình OLED
        if (xQueueReceive(temperaturaQueue, &temp, pdMS_TO_TICKS(1000)) == pdPASS) {
          if (!isnan(temp) && temp > 0) {
            Serial.print("Temperature: ");
            Serial.println(temp);
            updated = true;
          }
        }

        if (xQueueReceive(humidityQueue, &humi, pdMS_TO_TICKS(1000)) == pdPASS) {
          if (!isnan(humi) && humi > 0) {
            Serial.print("Humidity: ");
            Serial.println(humi);
            updated = true;
          }
        }

        if (xQueueReceive(airQueue, &air, pdMS_TO_TICKS(1000)) == pdPASS) {
          if (air > 0) {
            Serial.print("Air: ");
            Serial.println(air);
            updated = true;
          }
        }

        // Hiển thị dữ liệu nếu có sự thay đổi
        if (updated) {
          display.clearDisplay();  // Xóa màn hình cũ
          display.setCursor(10, 10);
          display.print("Temp: ");
          display.print(temp);
          display.println(" C");

          display.setCursor(10, 25);
          display.print("Humidity: ");
          display.print(humi);
          display.println(" %");

          display.setCursor(10, 40);
          display.print("Air: ");
          display.println(air);
          
          display.display();  // Cập nhật màn hình
        }
        break;

      case 2:
        display.clearDisplay();  // Xóa màn hình
        display.setCursor(20, 20);
        display.print("Hello");
        display.display();
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(200));  // Giảm tần suất kiểm tra nút
  }
}
