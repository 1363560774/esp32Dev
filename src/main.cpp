//////// #include <ESP8266WiFi.h>
//////#include <WiFi.h>
//////#include <PubSubClient.h>
//////
//////// WiFi
//////const char *ssid = "kai"; // Enter your WiFi name
//////const char *password = "4115241996...";  // Enter WiFi password
//////
//////// MQTT Broker
//////const char *mqtt_broker = "kai.zhaokai96.com";
//////const char *topic = "esp8266/test";
//////const char *mqtt_username = "admin";
//////const char *mqtt_password = "public";
//////const int mqtt_port = 1883;
//////
//////WiFiClient espClient;
//////PubSubClient client(espClient);
//////
//////void callback(char *topic, byte *payload, unsigned int length) {
//////    Serial.print("Message arrived in topic: ");
//////    Serial.println(topic);
//////    Serial.print("Message:");
//////    for (int i = 0; i < length; i++) {
//////        Serial.print((char) payload[i]);
//////    }
//////    Serial.println();
//////    Serial.println("-----------------------");
//////}
//////
//////void setup() {
//////    // Set software serial baud to 115200;
//////    Serial.begin(115200);
//////    // connecting to a WiFi network
//////    WiFi.begin(ssid, password);
//////    while (WiFi.status() != WL_CONNECTED) {
//////        delay(500);
//////        Serial.println("Connecting to WiFi..");
//////    }
//////    Serial.println("Connected to the WiFi network");
//////    //connecting to a mqtt broker
//////    client.setServer(mqtt_broker, mqtt_port);
//////    client.setCallback(callback);
//////    while (!client.connected()) {
//////        String client_id = "esp8266-client-";
//////        client_id += String(WiFi.macAddress());
//////        Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
//////        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
//////            Serial.println("Public emqx mqtt broker connected");
//////        } else {
//////            Serial.print("failed with state ");
//////            Serial.print(client.state());
//////            delay(2000);
//////        }
//////    }
//////    // publish and subscribe
//////    client.publish(topic, "hello emqx");
//////    client.subscribe(topic);
//////}
//////
//////void loop() {
//////    client.loop();
//////}
////
////#include <Arduino.h>
////
////// 引脚定义
////#define CLK_PIN 32
////#define DT_PIN 33
////#define SERVO_PIN 5 // 舵机连接的GPIO
////
////// 编码器计数变量
////volatile int counter = 0;
////
////// 中断服务函数
////void IRAM_ATTR isr_encoder() {
////
////    static int lastCLK = 0;
////    int currentCLK = digitalRead(CLK_PIN);
////
////    if (lastCLK == LOW && currentCLK == HIGH) {
////
////        if (digitalRead(DT_PIN) == HIGH) {
////            counter++;
////        } else {
////            counter--;
////        }
////
////    }
////
////    lastCLK = currentCLK;
////}
////
////void setup() {
////
////    Serial.begin(115200);
////    // 初始化舵机接口为输出
////    pinMode(SERVO_PIN, OUTPUT);
////    pinMode(CLK_PIN, INPUT_PULLUP);
////    pinMode(DT_PIN, INPUT_PULLUP);
////
////    // 设置中断,上升沿触发
////    attachInterrupt(CLK_PIN, isr_encoder, RISING);
////
////    // 初始化编码器计数
////    counter = 0;
////}
////
////void servoControl(int angle) {
////
////    int pulsewidth = map(angle, 0, 180, 500, 2500);
////
////    digitalWrite(SERVO_PIN, HIGH);
////    delayMicroseconds(pulsewidth);
////    digitalWrite(SERVO_PIN, LOW);
////
////    delay(20); //间隔20ms发送下一个脉冲
////}
////
////void loop() {
////
////    // 显示编码器计数
////    Serial.println(counter);
////    servoControl(counter);
////    // 小延时
//////    delay(1);
////}
//#include <Arduino.h>
//#include "driver/adc.h"
//
//void setup() {
//    Serial.begin(115200);
//
//    //配置 ADC
//    adc1_config_width(ADC_WIDTH_12Bit);
//    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
//}
//
//void loop() {
//    // 读取ADC值
//    int adc_value = adc1_get_raw(ADC1_CHANNEL_5);
//
//    // 将ADC值转换为电压值
//    double voltage = (3.3 / 4095.0) * adc_value;
//    double R = 1000.0 * adc_value / 4095 / 6;
//
//    // 打印输出
//    Serial.print(", R: ");
//    Serial.print(R);
//    Serial.print("ADC Value: ");
//    Serial.print(adc_value);
//    Serial.print(", Voltage: ");
//    Serial.print(voltage);
//    Serial.println(" V");
//
//    delay(500);
//}

#include <Arduino.h>
#include <ESP32Servo.h>
#define servoPin 5 // 舵机连接的GPIO
#define ENC_A_PIN 27 //编码器A相连接GPIO 27
#define ENC_B_PIN 26 //编码器B相连接GPIO 26

Servo myservo; // 创建Servo对象来控制舵机

volatile long encoderValue = 0; //编码器计数值

void IRAM_ATTR readEncoder(){
    int valA = digitalRead(ENC_A_PIN);
    int valB = digitalRead(ENC_B_PIN);

    if(valA == valB){
        encoderValue--;
    } else {
        encoderValue++;
    }
}

void servoControl(int angle) {

    int pulsewidth = map(angle, 0, 180, 500, 2500);

//    digitalWrite(SERVO_PIN, HIGH);
//    delayMicroseconds(pulsewidth);
//    digitalWrite(SERVO_PIN, LOW);

    delay(20); //间隔20ms发送下一个脉冲
}

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    //将引脚连接到Servo对象
    myservo.attach(servoPin);

    //中断在变化时读取编码器
    attachInterrupt(ENC_A_PIN, readEncoder, CHANGE);
}


void loop() {
    Serial.println(encoderValue); //打印编码器计数值
    myservo.write(encoderValue);
    delay(20);
}