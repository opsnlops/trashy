#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

void setup_ota(String hostname);
void start_ota();

/**
 * @brief A task to check for pending updates in a loop
 */
portTASK_FUNCTION_PROTO( creatureOTATask, pvParameters );
