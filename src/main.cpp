#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <cmath>
#include "FS.h"
#include "LittleFS.h"

#include "secret.h"

const int MQ3_PIN = 32;
const int MQ4_PIN = 33;
const int MQ8_PIN = 34;
const int MQ135_PIN = 35;

struct SensorCalibration
{
  float RL;
  float Ro;
  float RoCleanAir;
  float a;
  float b;
  float minValue;
  float maxValue;
};

SensorCalibration MQ3 = {15.0, 0.51, 9.21, -0.45, 1.4, 0.0, 1000.0};
SensorCalibration MQ4 = {20.0, 17.18, 4.4, -0.45, 1.4, 0.0, 1000.0};
SensorCalibration MQ8 = {10.0, 3.58, 9.21, -0.42, 1.3, 0.0, 1000.0};
SensorCalibration MQ135 = {20.0, 25.45, 3.6, -0.57, 1.488, 0.0, 1000.0};

const int NUM_FEATURES = 4;
const int NUM_CLASSES = 2;
const int K = 3;

struct Sample
{
  std::vector<float> features;
  int label;
};

std::vector<Sample> trainingData;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

TaskHandle_t SensorReadTaskHandle;
TaskHandle_t SensorProcessTaskHandle;

volatile float MQ3_Rs = 0.0, MQ4_Rs = 0.0, MQ8_Rs = 0.0, MQ135_Rs = 0.0;
volatile bool newReadingsAvailable = false;

void connectToWiFi();
void connectToMQTT();
float measureRs(int PIN, SensorCalibration sensor);
float getPPM(float ratio, float a, float b);
void initializeTrainingData();
std::vector<float> normalizeReadings(float mq3, float mq4, float mq8, float mq135);
int knnClassify(const std::vector<float> &sample);

void initializeTrainingData()
{
  trainingData.push_back({{100, 200, 150, 300}, 0});
  trainingData.push_back({{120, 180, 140, 280}, 0});
  trainingData.push_back({{110, 190, 145, 290}, 0});
  trainingData.push_back({{400, 600, 450, 800}, 1});
  trainingData.push_back({{450, 650, 500, 850}, 1});
  trainingData.push_back({{420, 630, 480, 820}, 1});
}

std::vector<float> normalizeReadings(float mq3, float mq4, float mq8, float mq135)
{
  std::vector<float> normalized(4);
  normalized[0] = (mq3 - MQ3.minValue) / (MQ3.maxValue - MQ3.minValue);
  normalized[1] = (mq4 - MQ4.minValue) / (MQ4.maxValue - MQ4.minValue);
  normalized[2] = (mq8 - MQ8.minValue) / (MQ8.maxValue - MQ8.minValue);
  normalized[3] = (mq135 - MQ135.minValue) / (MQ135.maxValue - MQ135.minValue);
  return normalized;
}

int knnClassify(const std::vector<float> &sample)
{
  std::vector<std::pair<float, int>> distances;

  for (const auto &train : trainingData)
  {
    float dist = 0;
    for (size_t i = 0; i < NUM_FEATURES; i++)
    {
      float diff = sample[i] - train.features[i];
      dist += diff * diff;
    }
    dist = sqrt(dist);
    distances.push_back({dist, train.label});
  }

  sort(distances.begin(), distances.end());

  int freshVotes = 0, spoiledVotes = 0;
  for (int i = 0; i < K; i++)
  {
    if (distances[i].second == 0)
      freshVotes++;
    else
      spoiledVotes++;
  }

  return (freshVotes > spoiledVotes) ? 0 : 1;
}

void SensorReadTask(void *parameter)
{
  while (true)
  {
    MQ3_Rs = measureRs(MQ3_PIN, MQ3);
    MQ4_Rs = measureRs(MQ4_PIN, MQ4);
    MQ8_Rs = measureRs(MQ8_PIN, MQ8);
    MQ135_Rs = measureRs(MQ135_PIN, MQ135);

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
      float MQ3_ratio = MQ3_Rs / MQ3.Ro;
      float MQ3_ppm = getPPM(MQ3_ratio, MQ3.a, MQ3.b);

      float MQ4_ratio = MQ4_Rs / MQ4.Ro;
      float MQ4_ppm = getPPM(MQ4_ratio, MQ4.a, MQ4.b);

      float MQ8_ratio = MQ8_Rs / MQ8.Ro;
      float MQ8_ppm = getPPM(MQ8_ratio, MQ8.a, MQ8.b);

      float MQ135_ratio = MQ135_Rs / MQ135.Ro;
      float MQ135_ppm = getPPM(MQ135_ratio, MQ135.a, MQ135.b);

      std::vector<float> normalized = normalizeReadings(MQ3_ppm, MQ4_ppm, MQ8_ppm, MQ135_ppm);

      int result = knnClassify(normalized);

      StaticJsonDocument<200> doc;
      doc["mq3"] = MQ3_ppm;
      doc["mq4"] = MQ4_ppm;
      doc["mq8"] = MQ8_ppm;
      doc["mq135"] = MQ135_ppm;
      doc["status"] = result == 0 ? "Fresh" : "Spoiled";

      char payload[200];
      serializeJson(doc, payload);

      mqttClient.publish(TOPIC_STATUS, payload);

      Serial.println(payload);

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

  initializeTrainingData();

  xTaskCreatePinnedToCore(
      SensorReadTask,
      "SensorReadTask",
      4096,
      NULL,
      1,
      &SensorReadTaskHandle,
      1);

  xTaskCreatePinnedToCore(
      SensorProcessTask,
      "SensorProcessTask",
      4096,
      NULL,
      1,
      &SensorProcessTaskHandle,
      1);
}

void loop()
{
  if (!mqttClient.connected())
  {
    connectToMQTT();
  }
  mqttClient.loop();
}

void connectToWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void connectToMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("FoodMonitorClient"))
    {
      Serial.println("Connected to MQTT");
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      delay(5000);
    }
  }
}

float measureRs(int PIN, SensorCalibration sensor)
{
  return 1.0;
}

float getPPM(float ratio, float a, float b)
{
  return a * pow(ratio, b);
}
