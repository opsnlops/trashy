// This isn't gonna work on anything but an ESP32
#if !defined(ESP32)
#error This code is intended to run only on the ESP32 board
#endif

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Adafruit_LC709203F.h>

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <time.h>
#include <sys/time.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
}

#include "creature.h"

#include "ota.h"

#include "mqtt/mqtt.h"
#include "network/connection.h"
#include "creatures/creatures.h"
#include "time/time.h"
#include "mdns/creature-mdns.h"
#include "mdns/magicbroker.h"

using namespace creatures;

static const char *TAG = "Main";

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

Adafruit_LC709203F battery;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    while (!Serial)
        ;

    delay(2000);
    ESP_LOGI(TAG, "Helllllo! I'm up and running on on %s!", ARDUINO_VARIANT);

    if (!battery.begin())
    {
        ESP_LOGE(TAG, "Unable to find the battery monitor!");
    }
    battery.setPackSize(LC709203F_APA_2000MAH);

    // MagicBroker::mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(MagicBroker::connectToMqtt));

    NetworkConnection network = NetworkConnection();
    network.connectToWiFi();

    // Register ourselves in mDNS
    CreatureMDNS creatureMDNS = CreatureMDNS(CREATURE_NAME);
    creatureMDNS.registerService(666);
    creatureMDNS.addStandardTags();

    digitalWrite(LED_BUILTIN, LOW);

    Time time = Time();
    time.init();
    time.obtainTime();

    // Get the location of the magic broker
    MagicBroker magicBroker;
    magicBroker.find();

    // Connect to MQTT
    MQTT mqtt = MQTT(String(CREATURE_NAME));
    mqtt.connect(magicBroker.ipAddress, magicBroker.port);
    mqtt.subscribe(String("cmd"), 0);
    mqtt.onMessage(onMqttMessage);

    // Enable OTA
    setup_ota(String(CREATURE_NAME));
    start_ota();

    mqtt.publish(String("status"), String("I'm alive!!"), 0, false);

    mqtt.startHeartbeat();
}

void loop()
{
    // Nothing! It's all handled from tasks
    // vTaskDelete(NULL);

    ESP_LOGI(TAG, "Battery voltage: %f", battery.cellVoltage());
    ESP_LOGI(TAG, "Battery percent: %f", battery.cellPercent());

    vTaskDelay(pdMS_TO_TICKS(10000));
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{

    ESP_LOGV(TAG, "LOCAL Message received.");
    digitalWrite(LED_BUILTIN, HIGH);

    // MQTT assumes it might be binary, which we don't do. We need to ensure there's a NUL at the end
    // of the message
    char payload_string[len + 1];
    memset(payload_string, '\0', len + 1);
    memcpy(payload_string, payload, len);

    ESP_LOGD(TAG, "Local MQTT Message: topic: %s, payload: %s", topic, payload_string);

    // Deserialize the JSON document
    DynamicJsonDocument doc(200);
    DeserializationError error = deserializeJson(doc, payload_string);

    // Test if parsing succeeds.
    if (error)
    {
        ESP_LOGE(TAG, "JSON decode failed: %s", error.f_str());
    }
    else
    {
        ESP_LOGI(TAG, "command: %s", doc["cmd"].as<String>());
    }

    digitalWrite(LED_BUILTIN, LOW);
}
