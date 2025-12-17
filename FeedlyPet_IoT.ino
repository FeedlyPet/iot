#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Preferences.h>

#define SERVO_PIN 18
#define TRIG_PIN 22
#define ECHO_PIN 23
#define BUTTON_PIN 4
#define BOOT_BUTTON 0
#define LED_PIN 2
#define MAX_DISTANCE 400
#define FULL_DISTANCE 5
#define EMPTY_DISTANCE 30
#define HEARTBEAT_INTERVAL 300000
#define FOOD_LEVEL_INTERVAL 600000
#define RECONNECT_DELAY 5000
#define DEBOUNCE_DELAY 50
#define BUTTON_HOLD_TIME 5000

WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Servo feedServo;
Preferences preferences;

char mqtt_server[40] = "192.168.0.102";
char mqtt_port[6] = "1883";
char device_id[32] = "string";
char mqtt_password[64] = "bPHSGi833i8WI28UOUBWrZUn6RpUYIbA";

bool mqttConnected = false;
int currentFoodLevel = 0;
int lastPublishedFoodLevel = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastFoodCheck = 0;
unsigned long lastButtonPress = 0;
bool buttonPressed = false;

unsigned long bootButtonPressStart = 0;
bool bootButtonPressed = false;
bool bootButtonHoldDetected = false;

String topicOnline;
String topicFood;
String topicFeeding;
String topicError;
String topicCommandFeed;
String topicCommandConfig;

WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 6);
WiFiManagerParameter custom_device_id("device", "Device ID", device_id, 32);
WiFiManagerParameter custom_mqtt_password("password", "MQTT Password", mqtt_password, 64);

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  
  return distance;
}

int getFoodLevel() {
  long distance = getDistance();
  
  if (distance == 0 || distance > MAX_DISTANCE) {
    Serial.println("Sensor error");
    return -1;
  }
  
  int level = map(distance, EMPTY_DISTANCE, FULL_DISTANCE, 0, 100);
  level = constrain(level, 0, 100);
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("cm → Level: ");
  Serial.print(level);
  Serial.println("%");
  
  return level;
}

String getTimestamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  
  if (!getLocalTime(&timeinfo)) {
    return "1970-01-01T00:00:00.000Z";
  }
  
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);
  return String(buffer);
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void blinkLEDFast(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

bool executeFeed(int portionSize) {
  Serial.println("\nFeeding started...");
  
  int duration = map(portionSize, 0, 100, 500, 1000);
  
  feedServo.write(90);
  Serial.println("Feeder OPEN");
  delay(duration);
  
  feedServo.write(0);
  Serial.println("Feeder CLOSED");
  
  Serial.println("Feeding complete\n");
  return true;
}

void publishOnlineStatus(bool online) {
  if (!mqttConnected) return;
  
  String payload = "{\"online\":" + String(online ? "true" : "false") + 
                   ",\"timestamp\":\"" + getTimestamp() + "\"}";
  
  mqttClient.publish(topicOnline.c_str(), payload.c_str(), true);
  
  Serial.print("Online: ");
  Serial.println(online ? "true" : "false");
}

void publishFoodLevel(int level) {
  if (!mqttConnected) return;
  
  String payload = "{\"deviceId\":\"" + String(device_id) + 
                   "\",\"level\":" + String(level) + 
                   ",\"timestamp\":\"" + getTimestamp() + "\"}";
  
  mqttClient.publish(topicFood.c_str(), payload.c_str());
  
  Serial.print("Food level: ");
  Serial.print(level);
  Serial.println("%");
}

void publishFeedingEvent(int portionSize, bool success, String type) {
  if (!mqttConnected) return;
  
  String payload = "{\"deviceId\":\"" + String(device_id) + 
                   "\",\"portionSize\":" + String(portionSize) + 
                   ",\"success\":" + String(success ? "true" : "false") + 
                   ",\"type\":\"" + type + 
                   "\",\"timestamp\":\"" + getTimestamp() + "\"}";
  
  mqttClient.publish(topicFeeding.c_str(), payload.c_str());
  
  Serial.print("Feeding: ");
  Serial.println(success ? "SUCCESS" : "FAILED");
}

void publishError(String errorCode, String errorMessage) {
  if (!mqttConnected) return;
  
  String payload = "{\"deviceId\":\"" + String(device_id) + 
                   "\",\"errorCode\":\"" + errorCode + 
                   "\",\"errorMessage\":\"" + errorMessage + 
                   "\",\"timestamp\":\"" + getTimestamp() + "\"}";
  
  mqttClient.publish(topicError.c_str(), payload.c_str());
  
  Serial.print("Error: ");
  Serial.println(errorCode);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("\nMessage: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(message);
  
  if (String(topic) == topicCommandFeed) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error && doc["type"] == "feed") {
      int portionSize = doc["portionSize"] | 50;
      
      Serial.print("Feed command: ");
      Serial.print(portionSize);
      Serial.println("g");
      
      int level = getFoodLevel();
      if (level < 10) {
        Serial.println("Food too low!");
        publishError("FOOD_EMPTY", "Container nearly empty");
        publishFeedingEvent(portionSize, false, "remote");
        return;
      }
      
      blinkLED(2);
      bool success = executeFeed(portionSize);
      publishFeedingEvent(portionSize, success, "remote");
      
      delay(1000);
      currentFoodLevel = getFoodLevel();
      publishFoodLevel(currentFoodLevel);
    }
  }
  else if (String(topic) == topicCommandConfig) {
    Serial.println("Config command");
    Serial.println(message);
  }
}

void reconnectMQTT() {
  if (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    
    if (mqttClient.connect(device_id, device_id, mqtt_password)) {
      Serial.println("Connected!");
      mqttConnected = true;
      digitalWrite(LED_PIN, HIGH);
      
      mqttClient.subscribe(topicCommandFeed.c_str());
      mqttClient.subscribe(topicCommandConfig.c_str());
      Serial.println("Subscribed");
      
      publishOnlineStatus(true);
      publishFoodLevel(currentFoodLevel);
      
    } else {
      Serial.print("Failed, rc=");
      Serial.println(mqttClient.state());
      mqttConnected = false;
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void saveConfigCallback() {
  Serial.println("Saving config...");
  
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(device_id, custom_device_id.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  
  preferences.begin("feedlypet", false);
  preferences.putString("mqtt_server", mqtt_server);
  preferences.putString("mqtt_port", mqtt_port);
  preferences.putString("device_id", device_id);
  preferences.putString("mqtt_password", mqtt_password);
  preferences.end();
  
  Serial.println("Config saved!");
}

void loadConfig() {
  preferences.begin("feedlypet", true);
  
  String saved_server = preferences.getString("mqtt_server", "");
  String saved_port = preferences.getString("mqtt_port", "1883");
  String saved_device = preferences.getString("device_id", "");
  String saved_password = preferences.getString("mqtt_password", "");
  
  if (saved_server.length() > 0) {
    saved_server.toCharArray(mqtt_server, 40);
    saved_port.toCharArray(mqtt_port, 6);
    saved_device.toCharArray(device_id, 32);
    saved_password.toCharArray(mqtt_password, 64);
    
    Serial.println("Loaded config:");
    Serial.print("Server: ");
    Serial.println(mqtt_server);
    Serial.print("Device: ");
    Serial.println(device_id);
  }
  
  preferences.end();
}

void checkAndPublishFoodLevel() {
  int level = getFoodLevel();
  
  if (level < 0) {
    publishError("SENSOR_ERROR", "Cannot read sensor");
    return;
  }
  
  int change = abs(level - lastPublishedFoodLevel);
  if (change >= 5) {
    publishFoodLevel(level);
    lastPublishedFoodLevel = level;
    
    if (level < 20) {
      Serial.println("Food low!");
      blinkLED(3);
    }
  }
  
  currentFoodLevel = level;
}

void enterConfigPortal() {
  Serial.println("\nEntering Config Portal...");

  if (mqttConnected) {
    publishOnlineStatus(false);
    mqttClient.disconnect();
    mqttConnected = false;
  }
  
  WiFiManagerParameter temp_mqtt_server("server", "MQTT Server", mqtt_server, 40);
  WiFiManagerParameter temp_mqtt_port("port", "MQTT Port", mqtt_port, 6);
  WiFiManagerParameter temp_device_id("device", "Device ID", device_id, 32);
  WiFiManagerParameter temp_mqtt_password("password", "MQTT Password", mqtt_password, 64);
  
  WiFiManager tempWM;
  tempWM.addParameter(&temp_mqtt_server);
  tempWM.addParameter(&temp_mqtt_port);
  tempWM.addParameter(&temp_device_id);
  tempWM.addParameter(&temp_mqtt_password);
  tempWM.setSaveConfigCallback(saveConfigCallback);
  tempWM.setConfigPortalTimeout(180);
  
  tempWM.setAPCallback([](WiFiManager *wm) {
    Serial.println("\Config Portal Active!");
    Serial.println("WiFi: FeedlyPet-Setup");
    Serial.println("URL: http://192.168.4.1");
    blinkLEDFast(10);
  });
  
  if (!tempWM.startConfigPortal("FeedlyPet-Setup")) {
    Serial.println("Config portal timeout");
  } else {
    Serial.println("Config portal closed");
    
    strcpy(mqtt_server, temp_mqtt_server.getValue());
    strcpy(mqtt_port, temp_mqtt_port.getValue());
    strcpy(device_id, temp_device_id.getValue());
    strcpy(mqtt_password, temp_mqtt_password.getValue());
    
    String devId = String(device_id);
    topicOnline = "feedlypet/" + devId + "/status/online";
    topicFood = "feedlypet/" + devId + "/status/food";
    topicFeeding = "feedlypet/" + devId + "/event/feeding";
    topicError = "feedlypet/" + devId + "/error";
    topicCommandFeed = "feedlypet/" + devId + "/command/feed";
    topicCommandConfig = "feedlypet/" + devId + "/command/config";
    
    mqttClient.setServer(mqtt_server, atoi(mqtt_port));
    
    Serial.println("\nConfiguration updated!");
    blinkLED(3);
  }
}

void handleFeedButton() {
  bool currentState = (digitalRead(BUTTON_PIN) == LOW);
  
  if (currentState && !buttonPressed && 
      (millis() - lastButtonPress > DEBOUNCE_DELAY)) {
    
    buttonPressed = true;
    lastButtonPress = millis();
    
    Serial.println("\nFeed button pressed!");
    
    int level = getFoodLevel();
    if (level < 10) {
      Serial.println("Food too low!");
      blinkLED(5);
      if (mqttConnected) {
        publishError("FOOD_EMPTY", "Manual blocked");
      }
      return;
    }
    
    blinkLED(2);
    bool success = executeFeed(50);
    
    if (mqttConnected) {
      publishFeedingEvent(50, success, "manual");
      delay(1000);
      currentFoodLevel = getFoodLevel();
      publishFoodLevel(currentFoodLevel);
    }
  }
  
  if (!currentState) {
    buttonPressed = false;
  }
}

void handleBootButton() {
  bool currentState = (digitalRead(BOOT_BUTTON) == LOW);

  if (currentState && !bootButtonPressed) {
    bootButtonPressed = true;
    bootButtonPressStart = millis();
    bootButtonHoldDetected = false;
    Serial.println("\nBOOT button pressed - hold 5 sec for Config Mode");
  }

  if (currentState && bootButtonPressed && !bootButtonHoldDetected) {
    unsigned long holdDuration = millis() - bootButtonPressStart;

    if (holdDuration >= 1000 && holdDuration < BUTTON_HOLD_TIME) {
      unsigned long currentSecond = holdDuration / 1000;
      static unsigned long lastPrintedSecond = 0;
      
      if (currentSecond != lastPrintedSecond) {
        Serial.print("Holding BOOT: ");
        Serial.print(currentSecond);
        Serial.print("/5 sec");

        for (unsigned long i = 0; i < currentSecond; i++) {
          Serial.print("●");
        }
        Serial.println();
        
        lastPrintedSecond = currentSecond;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
    }

    if (holdDuration >= BUTTON_HOLD_TIME) {
      bootButtonHoldDetected = true;
      
      Serial.println("CONFIG MODE ACTIVATED! Opening WiFi Portal...");

      for (int i = 0; i < 10; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        delay(50);
      }
      
      enterConfigPortal();
      
      bootButtonPressed = false;
      bootButtonHoldDetected = false;
      digitalWrite(LED_PIN, mqttConnected ? HIGH : LOW);
      return;
    }
  }
  
  if (!currentState && bootButtonPressed) {
    unsigned long holdDuration = millis() - bootButtonPressStart;
    
    if (!bootButtonHoldDetected) {
      Serial.print("BOOT released after");
      Serial.print(holdDuration);
      Serial.println("ms (too early for config)");
    }
    
    bootButtonPressed = false;
    bootButtonHoldDetected = false;
    digitalWrite(LED_PIN, mqttConnected ? HIGH : LOW);
  }
}

void handleSerial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "status") {
      Serial.println("\n========== STATUS ==========");
      Serial.print("WiFi: ");
      Serial.println(WiFi.status() == WL_CONNECTED ? "OK" : "NO");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("MQTT: ");
      Serial.println(mqttConnected ? "OK" : "NO");
      Serial.print("Server: ");
      Serial.println(mqtt_server);
      Serial.print("Device: ");
      Serial.println(device_id);
      Serial.print("Food: ");
      Serial.print(currentFoodLevel);
      Serial.println("%");
      Serial.print("Heap: ");
      Serial.print(ESP.getFreeHeap());
      Serial.println(" bytes");
      Serial.println("============================\n");
    }
    else if (cmd == "feed") {
      executeFeed(50);
      if (mqttConnected) publishFeedingEvent(50, true, "manual");
    }
    else if (cmd == "level") {
      int level = getFoodLevel();
      if (mqttConnected && level >= 0) publishFoodLevel(level);
    }
    else if (cmd == "config") {
      Serial.println("Opening config portal...");
      enterConfigPortal();
    }
    else if (cmd == "reset") {
      Serial.println("Reset WiFi...");
      wifiManager.resetSettings();
      preferences.clear();
      delay(1000);
      ESP.restart();
    }
    else if (cmd == "restart") {
      Serial.println("Restart...");
      ESP.restart();
    }
    else if (cmd == "help") {
      Serial.println("\n========== COMMANDS ==========");
      Serial.println("status   - System status");
      Serial.println("feed     - Test feeding");
      Serial.println("level    - Check food level");
      Serial.println("config   - Open config portal");
      Serial.println("reset    - Reset WiFi");
      Serial.println("restart  - Restart ESP32");
      Serial.println("help     - This help");
      Serial.println("==============================\n");
    }
    else {
      Serial.println("Unknown. Type 'help'");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("FeedlyPet IoT - ESP32 Feeder");

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);

  feedServo.attach(SERVO_PIN);
  feedServo.write(0);
  delay(500);
  Serial.println("Servo ready");

  loadConfig();
  
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_device_id);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setAPCallback([](WiFiManager *wm) {
    Serial.println("\nConfig mode!");
    Serial.println("WiFi: FeedlyPet-Setup");
    Serial.println("URL: http://192.168.4.1");
    blinkLED(5);
  });

  if (!wifiManager.autoConnect("FeedlyPet-Setup")) {
    Serial.println("Failed");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("WiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  configTime(7200, 3600, "pool.ntp.org");
  Serial.println("NTP synced");

  String devId = String(device_id);
  topicOnline = "feedlypet/" + devId + "/status/online";
  topicFood = "feedlypet/" + devId + "/status/food";
  topicFeeding = "feedlypet/" + devId + "/event/feeding";
  topicError = "feedlypet/" + devId + "/error";
  topicCommandFeed = "feedlypet/" + devId + "/command/feed";
  topicCommandConfig = "feedlypet/" + devId + "/command/config";
  Serial.println("Topics ready");

  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  
  currentFoodLevel = getFoodLevel();
  lastPublishedFoodLevel = currentFoodLevel;
  
  Serial.println("\nSetup complete!");
  blinkLED(3);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost");
    digitalWrite(LED_PIN, LOW);
    WiFi.reconnect();
    delay(5000);
    return;
  }

  if (!mqttClient.connected()) {
    mqttConnected = false;
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect > RECONNECT_DELAY) {
      reconnectMQTT();
      lastReconnect = millis();
    }
  } else {
    mqttClient.loop();
  }

  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    publishOnlineStatus(true);
    lastHeartbeat = millis();
  }

  if (millis() - lastFoodCheck > FOOD_LEVEL_INTERVAL) {
    checkAndPublishFoodLevel();
    lastFoodCheck = millis();
  }

  handleFeedButton();
  handleBootButton();

  handleSerial();
  
  delay(10);
}