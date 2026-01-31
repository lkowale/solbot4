#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include "Wservo.h"

// ********** RGB LED settings **********
#define LED_PIN 21    // LOLIN S3 Mini RGB LED pin
#define LED_COUNT 1
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

// ********** WiFi settings **********
const char* ssid = "solap";
const char* password = "solap1234";
// const char* ssid = "WLAN1";
// const char* password = "wlan1234";

// ********** MQTT settings **********
const char* mqtt_server = "192.168.0.190";
// const char* mqtt_server = "192.168.0.133";
const char* mqtt_user = "mark";
const char* mqtt_pass = "pass";
const int mqtt_port = 1883;
const char* deviceName = "FL";

WiFiClient mqttClient;
PubSubClient client(mqttClient);

Wservo servo(deviceName);

// Current EMA filter
static float Iavg = 0;

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

  // Handle PWM command - maps PWM (0-255) to speed
  strcpy(str, deviceName);
  strcat(str, "/pwm");
  if (topicStr == str) {
    int _pwm = doc["data"];
    servo.mode = servo.PWM;
    if (_pwm > 0)
      servo.move_forward(_pwm);
    else
      servo.move_backward(-_pwm);
    Serial.print(" move_pwm: "); Serial.print(_pwm);
  }

  // Handle set_speed command - speed in RPM
  strcpy(str, deviceName);
  strcat(str, "/set_speed");
  if (topicStr == str) {
    int _speed = doc["data"];
    servo.mode = servo.SPEED;
    servo.set_speed(_speed);  // RPM
    Serial.print(" set_speed: "); Serial.print(_speed);
  }

  // Handle direction command - 0=CW, 1=CCW
  strcpy(str, deviceName);
  strcat(str, "/direction");
  if (topicStr == str) {
    int _direction = doc["data"];
    servo.direction = _direction;
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
  Serial.println("ODrive Servo Controller Starting...");
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

  // Initialize servo (ODrive)
  servo.begin();
  // servo.debug = 1;  // Enable debug output

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

  // Servo update loop (100ms interval)
  static unsigned long servo_updateTime = millis();
  static const unsigned long SERVO_UPDATE_MS = servo.pid_update_window;
  if ((millis() - servo_updateTime) > SERVO_UPDATE_MS) {
    servo_updateTime = millis();
    servo.update();
  }

  // Telemetry publish loop (250ms interval)
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 250;
  if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    lastEventTime = millis();

    char str[80];
    String payload;

    // Publish position (placeholder - ODrive velocity mode doesn't track position)
    strcpy(str, deviceName);
    strcat(str, "/position");
    payload = String(servo.position);
    client.publish(str, payload.c_str());

    // Publish speed (RPM)
    strcpy(str, deviceName);
    strcat(str, "/speed");
    payload = String(servo.speed);
    client.publish(str, payload.c_str());

    // Publish PWM value (for compatibility)
    strcpy(str, deviceName);
    strcat(str, "/pwm_set");
    payload = String(servo.pwm);
    client.publish(str, payload.c_str());

    // Publish current (from ODrive) with EMA filter
    Iavg = 0.9f * Iavg + 0.1f * servo.current;
    strcpy(str, deviceName);
    strcat(str, "/current");
    payload = String(Iavg, 3);
    client.publish(str, payload.c_str());

    // Publish power (bus power)
    strcpy(str, deviceName);
    strcat(str, "/power");
    payload = String(servo.power, 2);
    client.publish(str, payload.c_str());

    // Publish bus voltage
    strcpy(str, deviceName);
    strcat(str, "/vbus");
    payload = String(servo.vbus_voltage, 2);
    client.publish(str, payload.c_str());

    // Publish FET temperature
    strcpy(str, deviceName);
    strcat(str, "/fet_temp");
    payload = String(servo.fet_temp, 1);
    client.publish(str, payload.c_str());

    // Publish servo occupancy (update execution time %)
    strcpy(str, deviceName);
    strcat(str, "/servo_occupancy");
    payload = String(servo.update_time_percent, 2);
    client.publish(str, payload.c_str());

    // Publish loop time percentage
    strcpy(str, deviceName);
    strcat(str, "/loop_time_percent");
    payload = String(loop_time_percent, 2);
    client.publish(str, payload.c_str());

    // Publish connection status
    strcpy(str, deviceName);
    strcat(str, "/connected");
    payload = String(servo.connected ? 1 : 0);
    client.publish(str, payload.c_str());

    // Publish speed goal (rev/s)
    strcpy(str, deviceName);
    strcat(str, "/speed_goal");
    payload = String(servo.speed_goal, 3);
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
