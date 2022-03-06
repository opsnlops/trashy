// This isn't gonna work on anything but an ESP32
#if !defined(ESP32)
#error This code is intended to run only on the ESP32 board
#endif

/*
    Big Note about Power Use!

    The ESP32 has a really touchy brownout detector. It'll trip pretty easily, which resets the
    board, and quickly.

    On this design, trying to use the WiFi and the sound card will almost always result in the
    brownout detector getting tripped, so I've taken care to make sure the sound card and WiFi
    chip are never used at the same time.

*/

/*

    Configuring

    The configuration is a JSON object that's read from MQTT:

    Sample config:

    {
        "volume": 20,
        "sleepTime": 3600,
        "version": "Feb 22, 2022 2:22:22am"
    }

    volume: The volume of the speaker that's passed to the mp3 player. Must be 0-30.
    sleepTime: Time in seconds to got into hibernation
    version: Doesn't really matter what it is, but a date is handy. If the version in MQTT doesn't
             match what's saved locally it'll overwrite what's saved on the chip with the MQTT version.

    This is stored as a Preference in the ESP32.
*/

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Adafruit_LC709203F.h>
#include <Preferences.h>

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
}

#include "creature.h"

#include "ota.h"

#include "logging/logging.h"
#include "mqtt/mqtt.h"
#include "network/connection.h"
#include "mdns/creature-mdns.h"
#include "creatures/creatures.h"

#include "DFRobot_DF1201S.h"

#define uS_TO_S_FACTOR 1000000

using namespace creatures;

// The lower this number the more sensitive
#define TOUCH_WAKEUP_THRESHOLD 0.1

String wakeupReason = "";

MQTT mqtt = MQTT(String(CREATURE_NAME));
Adafruit_LC709203F battery;
NetworkConnection network = NetworkConnection();
DFRobot_DF1201S player;
Preferences preferences;
Logger l = Logger();

// This is a battery powered device, don't go looking for the magic broker
// in mDNS.
IPAddress mqttAddress(10, 9, 1, 5);

// Configuration
uint8_t playerVolume;
uint64_t sleepTime;
String configVersion;

/*
    Function Prototypes
*/

void publishStateToMQTT(String status);
void doInitialBoot();
void goToSleep();
void setUpMQTT();
void touchCallback();
void doTouchEvent();
void publishBatteryState();
void playSound();
void setupSound();
void prepareForOTA();
void enableLDO2(boolean enabled);

void setup()
{
    // Before we do anything, get the logger going
    l.init();
    l.debug("Logging running!");

    // GPO 21 enables the second output
    pinMode(21, OUTPUT);
    enableLDO2(true);

    network.connectToWiFi();

    // Turn on the LED on the board so I can tell when it's awake or not
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Load the preferences
    preferences.begin(CREATURE_NAME);
    playerVolume = preferences.getUInt("volume", 20);
    sleepTime = preferences.getULong64("sleepTime", 3600 * 1); // One hour
    configVersion = preferences.getString("version", "NONE");
    preferences.end();

    // Put the amp to sleep if we had it open
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
        break;
    default:
        wakeupReason = "initial";
        doInitialBoot();
    }

    // Make sure sure the power is off
    enableLDO2(false);

    // Off to bed we go!
    goToSleep();
}

/**
 * @brief Configures the ESP32 to talk to the MP3 player via a hardware UART
 *
 * The MP3 players I use work at 115200 baud. The example docs show a software
 * UART, but we're running on a chip that has three hardware UARTs. Let's use
 * one of those instead to make it work better and reduce complexity.
 */
void setupSound()
{
    Serial1.begin(115200, SERIAL_8N1, 43, 44);
    while (!Serial1)
        ;
    l.info("Serial1 opened for the MP3 player");

    while (!player.begin(Serial1))
    {
        l.error("Unable to talk to the mp3 player");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // The player defaults to non-MUSIC, which makes all other API calls silently fail. I've
    // added some debugging to the player myself to make this more obvious.
    player.switchFunction(player.MUSIC);
    player.setPrompt(false);
    player.setPlayMode(player.SINGLE);
    l.debug("Play mode is currently %d (should be 3)", player.getPlayMode());

    // Get the volume from the flash (if I can figure that out)
    player.setVol(playerVolume);
    l.debug("VOL: %d", player.getVol());

    // Always leave the amp off when not using it, it's a huge power draw
    player.pause();
    player.setLED(false);
    player.disableAMP();
    l.info("Turned off the amp");
}

void playSound()
{
    // If this is on the player will announce what mode it's in when you
    // power it up. I don't want it to random say "MUSIC" out loud. :)
    player.setPrompt(false);

    // Turn the amp on for just a moment
    l.debug("Enabling the amp");
    player.setLED(true);
    player.enableAMP();

    // Play the first file
    l.info("Playing file");
    player.setPlayMode(player.SINGLE);
    player.playFileNum(0);

    vTaskDelay(pdMS_TO_TICKS(10000));

    l.debug("Disabling the amp");
    player.disableAMP();
    player.setLED(false);

    publishStateToMQTT("Played a sound");
}

void doInitialBoot()
{

    l.info("Helllllo! I'm up and running on on %s!", ARDUINO_VARIANT);

    if (!network.isConnected())
    {
        network.connectToWiFi();
    }

    publishBatteryState();
    publishStateToMQTT("Initial boot complete!");
}

void publishStateToMQTT(String status)
{

    // Connect to the Wifi if we haven't already
    if (!network.isConnected())
    {
        network.connectToWiFi();
    }

    // Get MQTT running
    setUpMQTT();

    // Publish our status to MQTT
    StaticJsonDocument<256> message;
    message["reason"] = wakeupReason;
    message["status"] = status;

    String json;
    serializeJson(message, json);
    mqtt.publish(String("status"), json, 1, false);

    // ...and the battery state
    publishBatteryState();
}

void publishBatteryState()
{

    if (battery.begin())
    {
        battery.setPackSize(LC709203F_APA_2000MAH);
        float cellVoltage = battery.cellVoltage();
        float cellPercent = battery.cellPercent();

        l.info("battery: voltage: %.2f, percent: %.2f", cellVoltage, cellPercent);

        StaticJsonDocument<130> message;
        message["voltage"] = String(cellVoltage);
        message["percent"] = String(cellPercent);

        String json;
        serializeJson(message, json);

        mqtt.publish(String("battery"), json, 0, true);
    }
    else
    {
        l.error("Unable to find the battery monitor!");
    }
}

void doTouchEvent()
{
    touch_pad_t touchPin = esp_sleep_get_touchpad_wakeup_status();

    switch (touchPin)
    {
    case TOUCH_PAD_NUM1:
        l.info("Touch detected on TOUCH_PAD_NUM1");
        playSound();
        break;
    default:
        l.warning("Woken up by a touch pin I don't know?! (%d)", touchPin);
        publishStateToMQTT("Unknown touchPin? Ut oh.");
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
    l.debug("Attempting to connect to MQTT");
    mqtt.connect(mqttAddress, 1883);
}

void goToSleep()
{
    l.debug("in goToSleep()");

    /*
        This is largely based on the docs:

        https://github.com/espressif/esp-idf/blob/master/examples/system/deep_sleep/main/deep_sleep_example_main.c

        The ESP32-S2 only supports one wakeup channel, unless the ESP32.
     */

    // Disable all of the sleep wakeup sources initially. We will then go turn on just the ones we need.
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // Wake up via the timer to update our battery status in Home Assistant
    esp_sleep_enable_timer_wakeup(sleepTime * uS_TO_S_FACTOR);

    /* Initialize touch pad peripheral. */
    touch_pad_init();

    // The ESP32-S2 only supports one sleep channel
    touch_pad_config(TOUCH_PAD_NUM1);

    // Set up a denoise filter (taken from the docs)
    touch_pad_denoise_t denoise = {
        .grade = TOUCH_PAD_DENOISE_BIT4,
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();
    l.debug("Denoise function init");

    // Filter settings (also taken from the docs)
    touch_filter_config_t filter_info = {
        .mode = TOUCH_PAD_FILTER_IIR_16,
        .debounce_cnt = 1, // 1 time count.
        .noise_thr = 0,    // 50%
        .jitter_step = 4,  // use for jitter mode.
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
    };
    touch_pad_filter_set_config(&filter_info);
    touch_pad_filter_enable();
    l.debug("touch pad filter init %d", TOUCH_PAD_FILTER_IIR_8);

    /* Set sleep touch pad. */
    touch_pad_sleep_channel_enable(TOUCH_PAD_NUM1, true);
    touch_pad_sleep_channel_enable_proximity(TOUCH_PAD_NUM1, false);

    /* Reducing the operating frequency can effectively reduce power consumption. */
    touch_pad_sleep_channel_set_work_time(1000, TOUCH_PAD_MEASURE_CYCLE_DEFAULT);

    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Calculate the threshold for when we should wake up
    uint32_t touchValue, wakeThreshold;
    touch_pad_sleep_channel_read_smooth(TOUCH_PAD_NUM1, &touchValue);
    wakeThreshold = touchValue * TOUCH_WAKEUP_THRESHOLD; // Set the threshold to trigger a wakeup

    touch_pad_sleep_set_threshold(TOUCH_PAD_NUM1, wakeThreshold);
    l.debug("Touch pad #%d average: %d, wakeup threshold set to %d",
            TOUCH_PAD_NUM1,
            touchValue,
            wakeThreshold);

    // Configure Touchpad as wakeup source
    esp_err_t WAKE_ON_TOUCHPAD = esp_sleep_enable_touchpad_wakeup();
    if (WAKE_ON_TOUCHPAD != ESP_OK)
    {
        l.warning("esp_sleep_enable_touchpad_wakeup() didn't turn ESP_OK: %d", WAKE_ON_TOUCHPAD);
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    l.info("Zzzzzzzzz z z z zzzz        z             z");
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (network.isConnected())
    {
        network.disconnectFromWiFi();
    }

    // Goodnight!
    esp_deep_sleep_start();
}

void loop()
{
    // This never gets called on battery powered devices
    vTaskDelete(NULL);
}

/**
 * @brief Prepare the ESP32 to be flashed via OTA
 *
 * This function sets up the system to be OTA flashed like a mains-powered creature, using
 * all of the normal stuff. :)
 */
void prepareForOTA()
{
    l.debug("Starting to get ready for OTA!");

    // Bring up the network
    if (!network.isConnected())
    {
        network.connectToWiFi();
    }
    l.debug("WiFi up!!");

    // Register in mDNS like normal
    CreatureMDNS creatureMDNS = CreatureMDNS(CREATURE_NAME, CREATURE_POWER);
    creatureMDNS.registerService(666);
    creatureMDNS.addStandardTags();
    l.debug("mDNS started!");

    // Enable OTA
    setup_ota(String(CREATURE_NAME));
    start_ota();
    l.debug("OTA is running!");

    int secondsToWait = 300;

    publishStateToMQTT("Waiting for OTA");

    // Wait for a few minutes for the OTA to happen. If the time expires, reboot.
    l.info("Ready for Over-The-Air update! I will reboot myself in %d seconds just in case.", secondsToWait);
    for (int i = 0; i < secondsToWait; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
        l.debug("Countdown: %d", (secondsToWait - i - 1));
    }

    l.warning("OTA timeout expired, rebooting!");
    ESP.restart();
}

void enableLDO2(boolean enabled)
{
    if (enabled)
    {
        l.debug("turning on the second v3.3 output");
        digitalWrite(21, HIGH);
    }
    else
    {
        l.debug("turning off the second v3.3 output");
        digitalWrite(21, LOW);
    }
}