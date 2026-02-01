#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include "Drv8871.h"

// ********** RGB LED settings **********
#define LED_PIN 21    // LOLIN S3 Mini RGB LED pin
#define LED_COUNT 1
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

// ********** WiFi settings **********
// const char* ssid = "solap";
// const char* password = "solap1234";
const char* ssid = "WLAN1";
const char* password = "wlan1234";

// ********** MQTT settings **********
// const char* mqtt_server = "192.168.0.190"; //t630
const char* mqtt_server = "192.168.0.121"; //hp-zbook
// const char* mqtt_server = "192.168.0.133";  //e7450
const char* mqtt_user = "mark";
const char* mqtt_pass = "pass";
const int mqtt_port = 1883;
const char* deviceName = "PL1";

WiFiClient mqttClient;
PubSubClient client(mqttClient);

Drv8871 motor(deviceName);

// Non-blocking MQTT reconnect - returns true if connected
bool tryReconnect() {
  static unsigned long lastAttempt = 0;
  unsigned long now = millis();

  // Try to connect every 5 seconds
  if (now - lastAttempt >= 5000 || lastAttempt == 0) {
    lastAttempt = now;
    Serial.print("Connecting to MQTT server...");
    if (client.connect(deviceName, mqtt_user, mqtt_pass)) {
      Serial.println("Connected!");

      // Solid green when connected
      led.setPixelColor(0, led.Color(0, 255, 0));
      led.show();

      char str[80];

      // Subscribe to control topics
      strcpy(str, deviceName);
      strcat(str, "/pwm");
      client.subscribe(str);

      strcpy(str, deviceName);
      strcat(str, "/set_speed");
      client.subscribe(str);

      strcpy(str, deviceName);
      strcat(str, "/direction");
      client.subscribe(str);

      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
  return false;
}

// Flash LED green at 1Hz when not connected to MQTT
void updateLedStatus() {
  static unsigned long lastFlash = 0;
  static bool ledOn = false;

  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastFlash >= 500) {
      lastFlash = now;
      ledOn = !ledOn;
      if (ledOn) {
        led.setPixelColor(0, led.Color(0, 255, 0));
      } else {
        led.setPixelColor(0, led.Color(0, 0, 0));
      }
      led.show();
    }
  }
}

void mqtt_callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" Message: ");
  String messageTemp;
  String topicStr = topic;

  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  StaticJsonDocument<256> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, messageTemp);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }

  char str[80];

  // Handle PWM command - direct PWM control (0-255)
  strcpy(str, deviceName);
  strcat(str, "/pwm");
  if (topicStr == str) {
    int _pwm = doc["data"];
    motor.mode = motor.PWM;
    if (_pwm > 0)
      motor.move_forward(_pwm);
    else
      motor.move_backward(-_pwm);
    Serial.print(" move_pwm: "); Serial.print(_pwm);
  }

  // Handle set_speed command - speed as percentage (-100 to 100)
  strcpy(str, deviceName);
  strcat(str, "/set_speed");
  if (topicStr == str) {
    int _speed = doc["data"];
    motor.mode = motor.SPEED;
    motor.set_speed(_speed);  // Percentage
    Serial.print(" set_speed: "); Serial.print(_speed);
  }

  // Handle direction command - 0=CW, 1=CCW
  strcpy(str, deviceName);
  strcat(str, "/direction");
  if (topicStr == str) {
    int _direction = doc["data"];
    motor.direction = _direction;
    Serial.print(" direction: "); Serial.print(_direction);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize RGB LED
  led.begin();
  led.setBrightness(128);  // Half brightness
  led.clear();
  led.show();
  delay(100);

  // Test flash - red on startup
  led.setPixelColor(0, led.Color(255, 0, 0));
  led.show();
  delay(500);

  Serial.println();
  Serial.println("DRV8871 DC Motor Controller Starting...");
  Serial.print("Device Name: ");
  Serial.println(deviceName);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // WiFi setup - flash red while connecting
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    // Flash red while connecting to WiFi
    led.setPixelColor(0, led.Color(255, 0, 0));
    led.show();
    delay(250);
    led.setPixelColor(0, led.Color(0, 0, 0));
    led.show();
    delay(250);
    Serial.println("Connecting to WiFi...");
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // MQTT setup
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);

  // Initialize DRV8871 motor driver
  motor.begin();
  // motor.debug = 1;  // Enable debug output

  Serial.println("Setup complete!");
}

void loop() {
  // Start timing loop execution
  unsigned long loop_start_us = micros();
  static unsigned long loop_time_ema_us = 0;
  static const float loop_time_alpha = 0.12f;
  static float loop_time_percent = 0.0f;

  // MQTT connection handling
  if (!client.connected())
    tryReconnect();
  client.loop();

  // Update LED status (flash when not connected)
  updateLedStatus();

  // Motor update loop (100ms interval)
  static unsigned long motor_updateTime = millis();
  static const unsigned long MOTOR_UPDATE_MS = motor.pid_update_window;
  if ((millis() - motor_updateTime) > MOTOR_UPDATE_MS) {
    motor_updateTime = millis();
    motor.update();
  }

  // Telemetry publish loop (250ms interval)
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 250;
  if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    lastEventTime = millis();

    char str[80];
    String payload;

    // Publish speed (as percentage)
    strcpy(str, deviceName);
    strcat(str, "/speed");
    payload = String(motor.speed);
    client.publish(str, payload.c_str());

    // Publish RPM from Hall sensors
    strcpy(str, deviceName);
    strcat(str, "/rpm");
    payload = String(motor.speed_rpm, 1);
    client.publish(str, payload.c_str());

    // Publish RPS from Hall sensors
    strcpy(str, deviceName);
    strcat(str, "/rps");
    payload = String(motor.speed_rps, 2);
    client.publish(str, payload.c_str());

    // Publish gear output RPM (raw)
    strcpy(str, deviceName);
    strcat(str, "/output_rpm");
    payload = String(motor.output_rpm, 1);
    client.publish(str, payload.c_str());

    // Publish filtered gear output RPM (used by PID)
    strcpy(str, deviceName);
    strcat(str, "/output_rpm_filtered");
    payload = String(motor.output_rpm_filtered, 1);
    client.publish(str, payload.c_str());

    // Publish gear output RPS
    strcpy(str, deviceName);
    strcat(str, "/output_rps");
    payload = String(motor.output_rps, 3);
    client.publish(str, payload.c_str());

    // Publish PWM value
    strcpy(str, deviceName);
    strcat(str, "/pwm_set");
    payload = String(motor.pwm);
    client.publish(str, payload.c_str());

    // Publish speed command (ramped)
    strcpy(str, deviceName);
    strcat(str, "/speed_cmd");
    payload = String(motor.speed_command);
    client.publish(str, payload.c_str());

    // Publish motor occupancy (update execution time %)
    strcpy(str, deviceName);
    strcat(str, "/motor_occupancy");
    payload = String(motor.update_time_percent, 2);
    client.publish(str, payload.c_str());

    // Publish loop time percentage
    strcpy(str, deviceName);
    strcat(str, "/loop_time_percent");
    payload = String(loop_time_percent, 2);
    client.publish(str, payload.c_str());

    // Publish connection status (always connected for DRV8871)
    strcpy(str, deviceName);
    strcat(str, "/connected");
    payload = String(motor.connected ? 1 : 0);
    client.publish(str, payload.c_str());

    // Publish speed goal (target output RPM in SPEED mode)
    strcpy(str, deviceName);
    strcat(str, "/speed_goal");
    payload = String(motor.speed_goal, 1);
    client.publish(str, payload.c_str());

    // Publish target output RPM
    strcpy(str, deviceName);
    strcat(str, "/target_rpm");
    payload = String(motor.target_output_rpm, 1);
    client.publish(str, payload.c_str());

    // Publish PID output
    strcpy(str, deviceName);
    strcat(str, "/pid_output");
    payload = String(motor.pid_output, 1);
    client.publish(str, payload.c_str());
  }

  // Compute loop EMA and percent relative to EVENT_INTERVAL_MS
  unsigned long loop_dur_us = micros() - loop_start_us;
  if (loop_time_ema_us == 0)
    loop_time_ema_us = loop_dur_us;
  else
    loop_time_ema_us = (unsigned long)(loop_time_alpha * (float)loop_dur_us + (1.0f - loop_time_alpha) * (float)loop_time_ema_us);

  float avg_ms = (float)loop_time_ema_us / 1000.0f;
  if (EVENT_INTERVAL_MS > 0)
    loop_time_percent = (avg_ms / (float)EVENT_INTERVAL_MS) * 100.0f;
  else
    loop_time_percent = 0.0f;
}
