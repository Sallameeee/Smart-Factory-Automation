#include <Ethernet.h>
#include <MQTT.h>
#include <DHT.h>

// DHT11 settings
#define DHTPIN 2  // Pin connected to the DHT11 sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Motor driver pins
#define MOTOR_IN1 3
#define MOTOR_IN2 4
#define MOTOR_PWM 5

// Other pins
#define PROXIMITY_PIN 8
#define BUZZER_PIN 6
#define GREEN_LED 9
#define RED_LED 10

#define TEMP_THRESHOLD 30.0 // Temperature alert threshold

// Ethernet settings
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};  // MAC address
byte ip[] = {192, 168, 1, 177};  // IP address (adjust to your network)
EthernetClient ethClient;
MQTTClient mqttClient;

// ThingSpeak settings
const char* thingSpeakHost = "api.thingspeak.com";
String thingSpeakAPIKey = "6LBHRT13OAIZKEAB";  // Your ThingSpeak API key
const int thingSpeakUpdateInterval = 16 * 1000;  // 16 seconds (to respect the 15-second update rule)
unsigned long lastThingSpeakUpdate = 0;

// MQTT settings
const char* mqttServer = "broker.mqtt.cool";
int mqttPort = 1883;
const char* mqttUsername = "public";
const char* mqttPassword = "public";
const char* mqttTopic = "/hello";

// Last MQTT publish time
unsigned long lastMqttPublish = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Start Ethernet connection
  Ethernet.begin(mac, ip);

  // Wait for Ethernet shield to initialize
  delay(1000);

  // Check if the shield is initialized correctly
  if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
    Serial.println("Failed to obtain IP address. Check network connection.");
    while (true);  // Keep the program running
  }

  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());

  // Initialize DHT sensor
  dht.begin();

  // Initialize motor and other pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(PROXIMITY_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Default state
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  stopMotor();

  // MQTT setup
  mqttClient.begin(mqttServer, ethClient);
  mqttClient.onMessage(messageReceived);
  connectMQTT();
}

void loop() {
  // Read temperature and humidity from DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read DHT sensor!");
    return;
  }

  // Read proximity sensor
  bool objectDetected = (digitalRead(PROXIMITY_PIN) == LOW);

  // Print readings to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Object Detected: ");
  Serial.println(objectDetected ? "Yes" : "No");

  // Temperature alert
  if (temperature > TEMP_THRESHOLD) {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Motor control
  if (objectDetected) {
    moveMotorForward();
  } else {
    stopMotor();
  }

  // Send data to ThingSpeak every 16 seconds (adhering to the 15-second rule)
  unsigned long currentMillis = millis();
  if (currentMillis - lastThingSpeakUpdate >= thingSpeakUpdateInterval) {
    lastThingSpeakUpdate = currentMillis;
    if (sendDataToThingSpeak(temperature, humidity, objectDetected)) {
      Serial.println("Data sent to ThingSpeak successfully.");
    } else {
      Serial.println("Failed to send data to ThingSpeak.");
    }
  }

  // MQTT loop
  mqttClient.loop();
  if (millis() - lastMqttPublish > 1000) {
    lastMqttPublish = millis();
    mqttClient.publish(mqttTopic, "world");
  }

  if (!mqttClient.connected()) {
    connectMQTT();
  }
}

// Function to send data to ThingSpeak
bool sendDataToThingSpeak(float temperature, float humidity, bool objectDetected) {
  if (ethClient.connect(thingSpeakHost, 80)) {
    String payload = "field1=" + String(temperature) +
                     "&field2=" + String(humidity) +
                     "&field3=" + String(objectDetected ? "1" : "0") +
                     "&api_key=" + thingSpeakAPIKey;

    // Send HTTP POST request using EthernetClient
    ethClient.println("POST /update HTTP/1.1");
    ethClient.println("Host: api.thingspeak.com");
    ethClient.println("Connection: close");
    ethClient.println("Content-Type: application/x-www-form-urlencoded");
    ethClient.print("Content-Length: ");
    ethClient.println(payload.length());
    ethClient.println();  // Blank line before the payload
    ethClient.print(payload);  // Send the payload
    
    // Wait for the response
    delay(500);  // Give the server time to respond
    if (ethClient.available()) {
      String response = "";
      while (ethClient.available()) {
        char c = ethClient.read();
        response += c;
      }
      Serial.println(response);  // Print the response for debugging
    }

    ethClient.stop();
    return true;
  }
  return false;
}

// MQTT connection function
void connectMQTT() {
  Serial.print("connecting to MQTT broker...");
  while (!mqttClient.connect("arduino", mqttUsername, mqttPassword)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected to MQTT broker!");
  mqttClient.subscribe(mqttTopic);
}

// MQTT message handler
void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

// Function to move motor forward
void moveMotorForward() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 128);  // Adjust speed as needed
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}
