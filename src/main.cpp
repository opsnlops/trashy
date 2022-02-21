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

#define uS_TO_S_FACTOR 1000000

using namespace creatures;

static const char *TAG = "Main";

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

MQTT mqtt = MQTT(String(CREATURE_NAME));
Adafruit_LC709203F battery;
NetworkConnection network = NetworkConnection();
Time creatureTime = Time();

void doWakeup();
void doInitialBoot();
void goToSleep();
void publishBatteryState();
void setUpMQTT();

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

#ifdef USE_SERIAL_PORT
    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    while (!Serial)
        ;

    delay(2000);
#endif

    esp_sleep_wakeup_cause_t wakeupReason;
    wakeupReason = esp_sleep_get_wakeup_cause();

    switch (wakeupReason)
    {
    case ESP_SLEEP_WAKEUP_TIMER:
        doWakeup();
        break;
    default:
        doInitialBoot();
    }
}

void doInitialBoot()
{

    ESP_LOGI(TAG, "Helllllo! I'm up and running on on %s!", ARDUINO_VARIANT);
    // MagicBroker::mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(MagicBroker::connectToMqtt));

    network.connectToWiFi();

    creatureTime.init();
    creatureTime.obtainTime();

    setUpMQTT();
    mqtt.publish(String("status"), String("I'm alive!!"), 0, false);

    publishBatteryState();

    goToSleep();
}

void doWakeup()
{

    network.connectToWiFi();

    setUpMQTT();
    publishBatteryState();

    goToSleep();
}

void publishBatteryState()
{

    setenv("TZ", DEFAULT_TIME_ZONE, 1);
    tzset();

    if (battery.begin())
    {
        battery.setPackSize(LC709203F_APA_2000MAH);
        StaticJsonDocument<130> message;
        message["voltage"] = String(battery.cellVoltage());
        message["percent"] = String(battery.cellPercent());
        message["updatedAt"] = Time::getCurrentTime();

        String json;
        serializeJson(message, json);

        mqtt.publish(String("battery"), json, 0, true);
    }
    else
    {
        ESP_LOGE(TAG, "Unable to find the battery monitor!");
    }
}

void setUpMQTT()
{
    // Connect to MQTT
    mqtt.connect(IPAddress(10, 9, 1, 5), 1883);
}

void goToSleep()
{

    int sleepTime = 30;
    esp_sleep_enable_timer_wakeup(sleepTime * uS_TO_S_FACTOR);
    ESP_LOGI(TAG, "Sleeping for %d seconds", sleepTime);

    /*
    Next we decide what all peripherals to shut down/keep on
    By default, ESP32 will automatically power down the peripherals
    not needed by the wakeup source,
    The line below turns off all RTC peripherals in deep sleep.
    */
    // esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    // Serial.println("Configured all RTC Peripherals to be powered down in sleep");

    /*
    Now that we have setup a wake cause and if needed setup the
    peripherals state in deep sleep, we can now start going to
    deep sleep.
    In the case that no wake up sources were provided but deep
    sleep was started, it will sleep forever unless hardware
    reset occurs.
    */

    esp_deep_sleep_start();
}

void loop()
{
    // This never gets called on battery powered devices
    vTaskDelete(NULL);
}
