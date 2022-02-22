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

#include "DFRobot_DF1201S.h"

#define uS_TO_S_FACTOR 1000000

using namespace creatures;

static const char *TAG = "Main";

// The higher this number the more sensitive
#define TOUCH_WAKEUP_THRESHOLD 50

// Keep track of how many times we've woken up and why
RTC_DATA_ATTR int wakeupCount = 0;
String wakeupReason = "";

// Store the current volume in the RTC so we don't have to go to MQTT
// for our config early
RTC_DATA_ATTR uint8_t playerVolume = 20;

MQTT mqtt = MQTT(String(CREATURE_NAME));
Adafruit_LC709203F battery;
NetworkConnection network = NetworkConnection();
Time creatureTime = Time();
DFRobot_DF1201S player;

void publishStateToMQTT(String status);
void doInitialBoot();
void goToSleep();
void setUpMQTT();
void touchCallback();
void doTouchEvent();
void publishBatteryState();
void playSound();
void setupSound();

void setup()
{
    // Mark that we woke up again
    wakeupCount++;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

#ifdef USE_SERIAL_PORT
    // Open serial communications and wait for port to open:
    Serial.begin(19200);
    while (!Serial)
        ;

    delay(2000);
#endif

    setupSound();

    switch (esp_sleep_get_wakeup_cause())
    {
    case ESP_SLEEP_WAKEUP_TIMER:
        wakeupReason = "timer";
        publishStateToMQTT("Wakeup due to timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        wakeupReason = "touched";
        doTouchEvent();
        publishStateToMQTT("Wakeup due to being touched");
        break;
    default:
        wakeupReason = "initial";
        doInitialBoot();
    }
}

void setupSound()
{
    Serial1.begin(115200, SERIAL_8N1, 16, 17);
    while (!Serial1)
        ;
    ESP_LOGI(TAG, "Serial1 opened for the MP3 player");

    while (!player.begin(Serial1))
    {
        ESP_LOGE(TAG, "Init failed, please check the wire connection!");
        delay(1000);
    }

    // The player defaults to non-MUSIC, which makes all other API calls silently fail. I've
    // added some debugging to the player myself to make this more obvious.
    player.switchFunction(player.MUSIC);
    player.setPrompt(false);
    player.setPlayMode(player.SINGLE);
    ESP_LOGD(TAG, "Play mode is currently %d (should be 3)", player.getPlayMode());

    // Get the volume from the RTC
    player.setVol(playerVolume);
    ESP_LOGD(TAG, "VOL: %d", player.getVol());

    // Always leave the amp off when not using it, it's a huge power draw
    player.pause();
    player.setLED(false);
    player.disableAMP();
    ESP_LOGI(TAG, "Turned off the amp");
}

void playSound()
{
    player.setPrompt(false);

    // Turn the amp on for just a moment
    ESP_LOGD(TAG, "Enabling the amp");
    player.setLED(true);
    player.enableAMP();

    // Play the first file
    ESP_LOGI(TAG, "Playing file");
    player.setPlayMode(player.SINGLE);
    player.playFileNum(0);

    vTaskDelay(pdMS_TO_TICKS(10000));
    //player.pause();

    ESP_LOGD(TAG, "Disabling the amp");
    player.disableAMP();
    player.setLED(false);
}

void doInitialBoot()
{

    ESP_LOGI(TAG, "Helllllo! I'm up and running on on %s!", ARDUINO_VARIANT);

    network.connectToWiFi();

    creatureTime.init();
    creatureTime.obtainTime();

    publishStateToMQTT("Initial Boot Completed");

    goToSleep();
}

void publishStateToMQTT(String status)
{

    setenv("TZ", DEFAULT_TIME_ZONE, 1);
    tzset();

    // Connect to the Wifi if we haven't already
    if (!network.isConnected())
    {
        network.connectToWiFi();
    }

    // Get MQTT running
    setUpMQTT();

    // Publish our status to MQTT
    StaticJsonDocument<256> message;
    message["wakeCount"] = wakeupCount;
    message["reason"] = wakeupReason;
    message["status"] = status;
    message["updatedAt"] = Time::getCurrentTime();

    String json;
    serializeJson(message, json);
    mqtt.publish(String("status"), json, 1, false);

    // ...and the battery state
    publishBatteryState();

    // Nicely hang up on the WiFi network
    network.disconnectFromWiFi();

    // Zzzzzz
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

void doTouchEvent()
{
    touch_pad_t touchPin = esp_sleep_get_touchpad_wakeup_status();

    switch (touchPin)
    {
    case 0:
        ESP_LOGI(TAG, "Touch detected on GPIO 4");
        playSound();
        break;
    case 1:
        ESP_LOGI(TAG, "Touch detected on GPIO 0");
        break;
    case 2:
        ESP_LOGI(TAG, "Touch detected on GPIO 2");
        break;
    case 3:
        ESP_LOGI(TAG, "Touch detected on GPIO 15");
        break;
    case 4:
        ESP_LOGI(TAG, "Touch detected on GPIO 13");
        break;
    case 5:
        ESP_LOGI(TAG, "Touch detected on GPIO 12");
        break;
    case 6:
        ESP_LOGI(TAG, "Touch detected on GPIO 14");
        break;
    case 7:
        ESP_LOGI(TAG, "Touch detected on GPIO 27");
        break;
    case 8:
        ESP_LOGI(TAG, "Touch detected on GPIO 33");
        break;
    case 9:
        ESP_LOGI(TAG, "Touch detected on GPIO 32");
        break;
    default:
        ESP_LOGI(TAG, "Wakeup not by touchpad");
        break;
    }
}

void touchCallback()
{
    // placeholder callback function
}

void setUpMQTT()
{
    // Connect to MQTT
    mqtt.connect(IPAddress(10, 9, 1, 5), 1883);
}

void goToSleep()
{

    int sleepTime = 1800;
    esp_sleep_enable_timer_wakeup(sleepTime * uS_TO_S_FACTOR);
    ESP_LOGI(TAG, "Sleeping for %d seconds", sleepTime);

    // Wake up on two touch points
    touchAttachInterrupt(OTA_GPIO, touchCallback, TOUCH_WAKEUP_THRESHOLD);    // Do the thing (blue wire)
    touchAttachInterrupt(ACTION_GPIO, touchCallback, TOUCH_WAKEUP_THRESHOLD); // Do OTA (yellow wire)

    // Configure Touchpad as wakeup source
    esp_sleep_enable_touchpad_wakeup();

    /*
    Next we decide what all peripherals to shut down/keep on
    By default, ESP32 will automatically power down the peripherals
    not needed by the wakeup source,
    The line below turns off all RTC peripherals in deep sleep.
    */
    // esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

    esp_deep_sleep_start();
}

void loop()
{
    // This never gets called on battery powered devices
    vTaskDelete(NULL);
}
