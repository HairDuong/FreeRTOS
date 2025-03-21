#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>

Servo myServo;
QueueHandle_t stateFanQueue;
QueueHandle_t statePumpQueue;
QueueHandle_t stateServorQueue;

// 🔹 **Định nghĩa chân GPIO**
#define TRIG_PIN1        18
#define ECHO_PIN1        19

#define PUMP_BUTTON_PIN  26  // Nút nhấn điều khiển máy bơm
#define FAN_BUTTON_PIN   33  // Nút nhấn điều khiển quạt
#define SERVO_BUTTON_PIN 32  // Nút nhấn điều khiển servo

#define PUMP_RELAY_PIN   27  // Relay điều khiển máy bơm
#define FAN_RELAY_PIN    25  // Relay điều khiển quạt
#define SERVO_PIN        17  // Chân điều khiển Servo



// 🔹 **Khai báo nguyên mẫu hàm**
void readUltrasonicSensor(void *pvParameters);
void handlePumpControl(void *pvParameters);
void handleServoControl(void *pvParameters);
void handleFanControl(void *pvParameters);

void setup() {
    Serial.begin(9600);
    myServo.attach(SERVO_PIN);
    myServo.write(90);  // Gán vị trí ban đầu cho Servo

    // 🔹 **Cấu hình chân I/O**
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT_PULLDOWN);

    pinMode(PUMP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FAN_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SERVO_BUTTON_PIN, INPUT_PULLUP);

    pinMode(PUMP_RELAY_PIN, OUTPUT);
    pinMode(FAN_RELAY_PIN, OUTPUT);

    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(FAN_RELAY_PIN, LOW);

    // Create Queue
    
    stateFanQueue = xQueueCreate(1, sizeof(char[4])); 


    // 🔹 **Tạo các task FreeRTOS**
    xTaskCreate(readUltrasonicSensor, "readUltrasonicSensor", 1000, NULL, 2, NULL);
    xTaskCreate(handlePumpControl, "handlePumpControl", 1000, NULL, 4, NULL);
    xTaskCreate(handleServoControl, "handleServoControl", 1000, NULL, 4, NULL);
    xTaskCreate(handleFanControl, "handleFanControl", 1000, NULL, 4, NULL);
}

void loop() {
    // Không làm gì trong loop vì dùng FreeRTOS
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
        }
        strcpy(statePumpQueueValue, isPumpOn ? "ON" : "OFF"); 
        xQueueSend(statePumpQueue, &statePumpQueueValue, portMAX_DELAY);
        
        
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
        }
        strcpy(stateServoQueueValue, isServoAt90 ? "ON" : "OFF"); 
        xQueueSend(stateServorQueue, &stateServoQueueValue, portMAX_DELAY);
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
            digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
            strcpy(stateFanQueueValue, isFanOn ? "ON" : "OFF"); 
            xQueueSend(stateFanQueue, &stateFanQueueValue, portMAX_DELAY);
        }

        lastFanButtonState = fanButtonState;
        vTaskDelay(pdMS_TO_TICKS(50));  // Debounce
    }
}
