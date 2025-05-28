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
#include <time.h>

Servo myServo;
QueueHandle_t stateFanQueue;
QueueHandle_t statePumpQueue;
QueueHandle_t stateServorQueue;
QueueHandle_t sensorQueue;
QueueHandle_t foodRateQueue;
QueueHandle_t hourQueue;
QueueHandle_t secondQueue;
QueueHandle_t minuteQueue;

TimerHandle_t xTimerSendSensor;
SemaphoreHandle_t xPumpMutex;
SemaphoreHandle_t xFanMutex;
SemaphoreHandle_t xPumpButtonsemaphore;
SemaphoreHandle_t xFanButtonsemaphore;
SemaphoreHandle_t xServoButtonsemaphore;
SemaphoreHandle_t xScreenButtonsemaphore;
SemaphoreHandle_t xModeButtonSemaphore;
volatile bool isAutoMode = true;  // Mặc định bật chế độ tự động

// Configuration for NTP Time
const long gmtOffset_sec = 7 * 3600;   
const int daylightOffset_sec = 0;
struct tm timeinfo;

 bool isPumpOn = false;
 bool isFanOn = false;
 bool isServoAt90 = false;




#define OLED_ADDR   0x3C  
#define DHTTYPE     DHT11 

#define DHTPIN      4     // Cảm biến DHT11
#define AIR_SENSOR  35    // Cảm biến khí (ADC)

#define CUSTOM_SDA  21    // I2C OLED SDA
#define CUSTOM_SCK  22    // I2C OLED SCL

#define OLED_BUTTON_PIN  16  // Nút nhấn OLED
#define MODE_BUTTON_PIN  15 // chuyen doi auto sang thu cong

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
void automaticfeeding(void *pvParameters);
void vHandleModeButton(void *pvParameters);
void reconnectMQTT() ;


// Định nghĩa cấu trúc chứa cả 3 giá trị
struct SensorData {
    float temperature;
    float humidity;
    float air;
};

void autoControlDevice(const SensorData &data);
void vReceiverSensor(void *pvParameters);
void vSender(TimerHandle_t xTimerSendSensor);
void IRAM_ATTR buttonISR();

Adafruit_SSD1306 display(128, 64, &Wire, -1);
DHT dht(DHTPIN, DHTTYPE);



void setup() {
    Serial.begin(9600);
    myServo.attach(SERVO_PIN);
    myServo.write(0);  // Gán vị trí ban đầu cho Servo
    Wire.begin();
    dht.begin();
    pinMode(AIR_SENSOR, INPUT); 

    
  xPumpButtonsemaphore = xSemaphoreCreateBinary();
  xFanButtonsemaphore = xSemaphoreCreateBinary();
  xServoButtonsemaphore = xSemaphoreCreateBinary();
  xScreenButtonsemaphore = xSemaphoreCreateBinary();
  xModeButtonSemaphore = xSemaphoreCreateBinary();
  


  attachInterrupt(digitalPinToInterrupt(PUMP_BUTTON_PIN), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN_BUTTON_PIN), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SERVO_BUTTON_PIN), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(OLED_BUTTON_PIN), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON_PIN), buttonISR, FALLING);
    


    // 🔹 **Cấu hình chân I/O**
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT_PULLDOWN);

    pinMode(PUMP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FAN_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SERVO_BUTTON_PIN, INPUT_PULLUP);

    pinMode(PUMP_RELAY_PIN, OUTPUT);
    pinMode(FAN_RELAY_PIN, OUTPUT);
    pinMode(OLED_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);

    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FAN_RELAY_PIN, LOW);

    // Create Queue
    
    stateFanQueue = xQueueCreate(1, sizeof(char[4])); 
    statePumpQueue = xQueueCreate(1, sizeof(char[4]));
    stateServorQueue = xQueueCreate(1, sizeof(char[4]));
    sensorQueue = xQueueCreate(3, sizeof(SensorData));  // Queue chứa 3 phần tử, mỗi phần tử là SensorData
    foodRateQueue = xQueueCreate(2, sizeof(float));
    hourQueue = xQueueCreate(1, sizeof(int));
    secondQueue = xQueueCreate(1, sizeof(int));
    minuteQueue = xQueueCreate(1, sizeof(int));

    // creat timer
    xTimerSendSensor = xTimerCreate ("timer send sensor", pdMS_TO_TICKS(3000), pdTRUE, (void *) 0, vSender  );
    if (xTimerSendSensor != NULL) {
    xTimerStart(xTimerSendSensor, 0); // Bắt đầu chạy timer
  } else {
    Serial.println("Timer tạo thất bại!");
  }
    // mutex create
     xPumpMutex = xSemaphoreCreateMutex();
     xFanMutex = xSemaphoreCreateMutex();

    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
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
    xTaskCreate(handlePumpControl, "handlePumpControl", 8192, NULL, 2, NULL);
    xTaskCreate(handleServoControl, "handleServoControl", 8192, NULL, 2, NULL);
    xTaskCreate(handleFanControl, "handleFanControl", 8192, NULL, 2, NULL);
    xTaskCreate( mqttLoopTask, "mqttLoopTask", 8192, NULL, 3, NULL);
    xTaskCreate(vReceiverSensor, "Receiver", 8192, NULL, 2, NULL);
    xTaskCreate(automaticfeeding, "automaticfeeding", 8192, NULL, 2, NULL);
    xTaskCreate(vHandleModeButton, "automaticfeeding", 8192, NULL, 2, NULL);

    
}

void loop() {
    // Không làm gì trong loop vì dùng FreeRTOS
}

void mqttLoopTask(void *pvParameters) {
  for (;;) {
    if (!client.connected()) {
      reconnectMQTT(); // Gọi lại hàm connect nếu mất kết nối
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("MQTT disconnected. Reconnecting...");
    String clientId = "esp32-client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Reconnected to MQTT!");
        client.subscribe(topic4);
  client.subscribe(topic5);
  client.subscribe(topic6);
  client.subscribe(topic7);
  client.subscribe(topic8);
  client.subscribe(topic9);
  client.subscribe(topic15);
  client.subscribe(topic16);
  client.subscribe(topic17);
    } else {
      Serial.print("MQTT reconnect failed. State=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// 🔹 **Task đọc cảm biến siêu âm**
void readUltrasonicSensor(void *pvParameters) {
    long duration1;
    float distanceOrigin = 8.0;
    float foodRateValue= 0.0;

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
        foodRateValue = foodAvailable;
        client.publish(topic3, String(foodAvailable).c_str());
        xQueueSend(foodRateQueue, &foodRateValue, 0);
        Serial.print("Food rate: ");
        Serial.println(foodAvailable);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// 🔹 **Task điều khiển máy bơm**
void handlePumpControl(void *pvParameters) 
{
      char statePumpQueueValue[4];
      for (;;) 
      {

        
           if (xSemaphoreTake(xPumpButtonsemaphore, portMAX_DELAY))
          {
            if( xSemaphoreTake(xPumpMutex, portMAX_DELAY) == pdTRUE) 
             {
        
            isPumpOn = !isPumpOn;
            digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
            client.publish(topic5, isPumpOn ? "ON" : "OFF"); 
            strcpy(statePumpQueueValue, isPumpOn ? "ON" : "OFF"); 
            xQueueSend(statePumpQueue, &statePumpQueueValue, 0);
             }
          xSemaphoreGive(xPumpMutex);
        }
          
      }
}

// 🔹 **Task điều khiển Servo**
void handleServoControl(void *pvParameters) {
   
    
    char stateServoQueueValue[4];

    for (;;) {
        if (xSemaphoreTake(xServoButtonsemaphore, portMAX_DELAY))
        {
        

        
            isServoAt90 = !isServoAt90;
            myServo.write(isServoAt90 ? 90 : 0);
            strcpy(stateServoQueueValue, isServoAt90 ? "ON" : "OFF"); 
            xQueueSend(stateServorQueue, &stateServoQueueValue, 0);
            client.publish(topic7, isServoAt90 ? "ON" : "OFF");
        
   
        }
    }
}
 
// 🔹 **Task điều khiển quạt**
void handleFanControl(void *pvParameters) {
  
    char stateFanQueueValue[4];

    for (;;) {
       
            if (xSemaphoreTake(xFanButtonsemaphore, portMAX_DELAY))
            {
               if( xSemaphoreTake(xFanMutex, portMAX_DELAY) == pdTRUE) 
           {
              isFanOn = !isFanOn;
              
              digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
              strcpy(stateFanQueueValue, isFanOn ? "ON" : "OFF"); 
              client.publish(topic4, isFanOn ? "ON" : "OFF");
              xQueueSend(stateFanQueue, &stateFanQueueValue, 0);
            }
            xSemaphoreGive(xFanMutex);
          }
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    String message;

    char statePumpQueueValue[4];
    char stateServoQueueValue[4];
    char stateFanQueueValue[4];
    int targetHour = 0;
    int targetMinute = 0;
    int targetSecond = 0;
    int targetSecondclose = 0;
    for (int i = 0; i < length; i++) message += (char)payload[i];

    if (String(topic) == topic6) isAutoMode = (message == "ON");
    if (String(topic) == topic4) isFanOn = (message == "ON");
    if (String(topic) == topic5) isPumpOn = (message == "ON");
    if (String(topic) == topic7) {
        isServoAt90 = (message == "ON");
        myServo.write(isServoAt90 ? 90 : 0);
    }
   if (String(topic) == topic15) {
  int targetHour = message.toInt();
  xQueueOverwrite(hourQueue, &targetHour);
  Serial.printf("Đã nhận targetHour: %d\n", targetHour);
}

if (String(topic) == topic16) {
  int targetMinute = message.toInt();
  xQueueOverwrite(minuteQueue, &targetMinute);
  Serial.printf("Đã nhận targetMinute: %d\n", targetMinute);
}

if (String(topic) == topic17) {
  int targetSecond = message.toInt();
  xQueueOverwrite(secondQueue, &targetSecond);
  Serial.printf("Đã nhận targetSecond: %d\n", targetSecond);
}

    digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
    digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
    
    strcpy(statePumpQueueValue, isPumpOn ? "ON" : "OFF"); 
        xQueueSend(statePumpQueue, &statePumpQueueValue, 0);
    strcpy(stateServoQueueValue, isServoAt90 ? "ON" : "OFF"); 
        xQueueSend(stateServorQueue, &stateServoQueueValue, 0);
    strcpy(stateFanQueueValue, isFanOn ? "ON" : "OFF"); 
        xQueueSend(stateFanQueue, &stateFanQueueValue, 0);  
     
    
  }
    

  
  void vSender(TimerHandle_t xTimerSendSensor) {    
  
    
        SensorData data;
        // Đọc dữ liệu từ các cảm biến
        data.temperature = dht.readTemperature();
        data.humidity = dht.readHumidity();
        data.air = analogRead(AIR_SENSOR);
        client.publish(topic0, String(data.temperature).c_str());
        client.publish(topic1, String(data.humidity).c_str());
        client.publish(topic2, String(data.air).c_str());

        // Kiểm tra dữ liệu hợp lệ trước khi gửi
        if (!isnan(data.temperature) && data.temperature > 0 &&
            !isnan(data.humidity) && data.humidity > 0 &&
            data.air > 0) {
            xQueueSend(sensorQueue, &data, portMAX_DELAY);
        }
                 autoControlDevice(data);     
    
    }

void vReceiverSensor(void *pvParameters) {
    SensorData sensorData = {0, 0, 0};
    char statePump[4];
    char stateFan[4];
    char stateServo[4];
           
    
    int currentScreen = 0;

    for (;;) {
           
            xQueueReceive(sensorQueue, &sensorData, pdMS_TO_TICKS(1000)) ;
        

        if (xSemaphoreTake(xScreenButtonsemaphore, 0) == pdTRUE) {
         currentScreen = (currentScreen + 1) % 2;
        }

        
        display.clearDisplay();

        if (currentScreen == 0) {
           xQueueReceive(statePumpQueue, &statePump, 0);
            xQueueReceive(stateFanQueue, &stateFan, 0);
            xQueueReceive(stateServorQueue, &stateServo, 0);

            

           

            display.setCursor(10, 20);
            display.print("Pump: "); display.print(statePump);

            display.setCursor(10, 35);
            display.print("Fan: "); display.print(stateFan);

            display.setCursor(10, 50);
            display.print("Servo: "); display.print(stateServo);
        } else {
            float foodRate= 0.0;
           
                display.setCursor(10, 20);
                display.print("Temp: "); display.print(sensorData.temperature); display.println(" C");

                display.setCursor(10, 30);
                display.print("Humidity: "); display.print(sensorData.humidity); display.println(" %");

                display.setCursor(10, 40);
                display.print("Air: "); display.println(sensorData.air);
            
            xQueueReceive(foodRateQueue, &foodRate, pdMS_TO_TICKS(1000)) ;
            
                display.setCursor(10, 50); display.print("food: "); display.print(foodRate); display.print(" %");
            
        }

        display.display();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vHandleModeButton(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xModeButtonSemaphore, portMAX_DELAY) == pdTRUE) {
      // Chuyển trạng thái chế độ
     
      isAutoMode = !isAutoMode;
      client.publish(topic6, isAutoMode ? "ON":"OFF");
      Serial.print(" chuyen mode thanh cong");
    }
  }
}


void autoControlDevice(const SensorData &data) {
  if (!isAutoMode) return;
    char stateFanQueueValue[4];
    char statePumpQueueValue[4];
 
    if (data.temperature > 30 ) {
      if (xSemaphoreTake(xFanMutex, portMAX_DELAY) == pdTRUE) 
      {
        Serial.println("⚠️ Cảnh báo: Nhiệt độ vượt ngưỡng!");
      
        client.publish(topic4, "ON" );
        digitalWrite(FAN_RELAY_PIN, HIGH );
        strcpy(stateFanQueueValue,  "ON" ); 
        xQueueSend(stateFanQueue, &stateFanQueueValue, 0);
      }
      xSemaphoreGive(xFanMutex);
    }
    

    if (data.air > 1000) {
      if (xSemaphoreTake(xPumpMutex, portMAX_DELAY) == pdTRUE) 
      {
        Serial.println("⚠️ Cảnh báo: Chất lượng không khí vượt ngưỡng!");
       
        strcpy(statePumpQueueValue,  "ON" ); 
        xQueueSend(statePumpQueue, &statePumpQueueValue, 0);    
        digitalWrite(PUMP_RELAY_PIN,  HIGH );
        client.publish(topic5,  "ON" ); 
      } 
      xSemaphoreGive(xPumpMutex);   
    } 
}



void automaticfeeding(void *pvParameters) {
  int targetHour = 0;
  int targetMinute = 0;
  int targetSecond = 0;
  int targetSecondclose = 0;
  int prevHour = -1, prevMinute = -1, prevSecond = -1;

  for (;;) {
    // Lấy thời gian thực từ NTP
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Không thể lấy thời gian từ NTP Server");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    // Nhận giá trị từ Queue nếu có thay đổi
    if (xQueueReceive(hourQueue, &targetHour, 0) == pdPASS ||
        xQueueReceive(minuteQueue, &targetMinute, 0) == pdPASS ||
        xQueueReceive(secondQueue, &targetSecond, 0) == pdPASS) {
      
      targetSecondclose = targetSecond + 7;
      if (targetSecondclose >= 60) targetSecondclose -= 60; // Xử lý tràn giây

      Serial.print("Thời gian cho ăn đã được đặt: ");
      Serial.printf("%02d:%02d:%02d (Đóng lại lúc %02d)\n", 
        targetHour, targetMinute, targetSecond, targetSecondclose);
    }

    // In ra thời gian hiện tại (1 lần mỗi giây)
    if (timeinfo.tm_sec != prevSecond) {
      prevHour = timeinfo.tm_hour;
      prevMinute = timeinfo.tm_min;
      prevSecond = timeinfo.tm_sec;

      Serial.printf("Thời gian hiện tại: %02d:%02d:%02d\n", 
        prevHour, prevMinute, prevSecond);
    }

    // Nếu đúng giờ mở nắp (servo về 0)
    if (timeinfo.tm_hour == targetHour &&
        timeinfo.tm_min == targetMinute &&
        timeinfo.tm_sec == targetSecond) {
      
      if (myServo.read() != 90) {
        myServo.write(90);
        
      }
    }

    // Nếu đúng giờ đóng nắp (servo về 90)
    if (timeinfo.tm_hour == targetHour &&
        timeinfo.tm_min == targetMinute &&
        timeinfo.tm_sec == targetSecondclose) {
      
      if (myServo.read() != 0) {
        myServo.write(0);
       
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Lặp lại sau 500ms
  }
}

void IRAM_ATTR buttonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Kiểm tra trạng thái các nút
  if (digitalRead(PUMP_BUTTON_PIN) == LOW) {
    xSemaphoreGiveFromISR(xPumpButtonsemaphore, &xHigherPriorityTaskWoken);
  }
  if (digitalRead(FAN_BUTTON_PIN) == LOW) {
    xSemaphoreGiveFromISR(xFanButtonsemaphore, &xHigherPriorityTaskWoken);
  }
   if (digitalRead(SERVO_BUTTON_PIN) == LOW) {
    xSemaphoreGiveFromISR(xServoButtonsemaphore, &xHigherPriorityTaskWoken);
  }
  if (digitalRead(OLED_BUTTON_PIN) == LOW) {
    xSemaphoreGiveFromISR(xScreenButtonsemaphore, &xHigherPriorityTaskWoken);
  }
  if (digitalRead(MODE_BUTTON_PIN) == LOW) {
    xSemaphoreGiveFromISR(xModeButtonSemaphore, &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}