#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "secret.h"

const int MQ3_PIN = 32;
const float MQ3_RL = 15.0;
const float MQ3_RO_CLEAN_AIR_RATIO = 9.21;
const float MQ3_a = -0.45;
const float MQ3_b = 1.4;
float MQ3_Ro = 0.51;

const int MQ4_PIN = 33;
const float MQ4_RL = 20.0;
const float MQ4_RO_CLEAN_AIR_RATIO = 4.4;
const float MQ4_a = -0.45;
const float MQ4_b = 1.4;
float MQ4_Ro = 17.18;

const int MQ8_PIN = 34;
const float MQ8_RL = 10.0;
const float MQ8_RO_CLEAN_AIR_RATIO = 9.21;
const float MQ8_a = -0.42;
const float MQ8_b = 1.3;
float MQ8_Ro = 3.58;

const int MQ135_PIN = 35;
const float MQ135_RL = 20.0;
const float MQ135_RO_CLEAN_AIR_RATIO = 3.6;
const float MQ135_a = -0.57;
const float MQ135_b = 1.488;
float MQ135_Ro = 25.45;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

TaskHandle_t SensorReadTaskHandle;
TaskHandle_t SensorProcessTaskHandle;

volatile float MQ3_Rs = 0.0, MQ4_Rs = 0.0, MQ8_Rs = 0.0, MQ135_Rs = 0.0;
volatile bool newReadingsAvailable = false;

void connectToWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void connectToMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32_Client", MQTT_USERNAME, MQTT_PASSWORD))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

float measureRs(int PIN, float RL)
{
  int sensorValue = analogRead(PIN);
  float sensorVoltage = sensorValue * (3.3 / 4095.0);
  return (3.3 - sensorVoltage) / sensorVoltage * RL;
}

float getPPM(float ratio, float a, float b)
{
  return pow(10, (log10(ratio) - b) / a);
}

void SensorReadTask(void *parameter)
{
  while (true)
  {
    MQ3_Rs = measureRs(MQ3_PIN, MQ3_RL);
    MQ4_Rs = measureRs(MQ4_PIN, MQ4_RL);
    MQ8_Rs = measureRs(MQ8_PIN, MQ8_RL);
    MQ135_Rs = measureRs(MQ135_PIN, MQ135_RL);

    newReadingsAvailable = true;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void SensorProcessTask(void *parameter)
{
  while (true)
  {
    if (newReadingsAvailable)
    {
      float MQ3_ratio = MQ3_Rs / MQ3_Ro;
      float MQ3_ppm = getPPM(MQ3_ratio, MQ3_a, MQ3_b);

      float MQ4_ratio = MQ4_Rs / MQ4_Ro;
      float MQ4_ppm = getPPM(MQ4_ratio, MQ4_a, MQ4_b);

      float MQ8_ratio = MQ8_Rs / MQ8_Ro;
      float MQ8_ppm = getPPM(MQ8_ratio, MQ8_a, MQ8_b);

      float MQ135_ratio = MQ135_Rs / MQ135_Ro;
      float MQ135_ppm = getPPM(MQ135_ratio, MQ135_a, MQ135_b);

      char payload[32];
      snprintf(payload, sizeof(payload), "%.2f", MQ3_ppm);
      mqttClient.publish(TOPIC_MQ3, payload);

      snprintf(payload, sizeof(payload), "%.2f", MQ4_ppm);
      mqttClient.publish(TOPIC_MQ4, payload);

      snprintf(payload, sizeof(payload), "%.2f", MQ8_ppm);
      mqttClient.publish(TOPIC_MQ8, payload);

      snprintf(payload, sizeof(payload), "%.2f", MQ135_ppm);
      mqttClient.publish(TOPIC_MQ135, payload);

      Serial.printf("MQ3: %.2f ppm, MQ4: %.2f ppm, MQ8: %.2f ppm, MQ135: %.2f ppm\n",
                    MQ3_ppm, MQ4_ppm, MQ8_ppm, MQ135_ppm);

      newReadingsAvailable = false;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  connectToWiFi();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  xTaskCreatePinnedToCore(SensorReadTask, "SensorReadTask", 4096, NULL, 1, &SensorReadTaskHandle, 1);
  xTaskCreatePinnedToCore(SensorProcessTask, "SensorProcessTask", 4096, NULL, 1, &SensorProcessTaskHandle, 1);
}

void loop()
{
  if (!mqttClient.connected())
  {
    connectToMQTT();
  }
  mqttClient.loop();
}
