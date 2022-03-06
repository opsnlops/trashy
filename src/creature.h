#pragma once

#define CREATURE_NAME "trashy"
#define CREATURE_POWER "battery"

// Toss some things into the global namespace so that the libs can read it
const char* gCreatureName = CREATURE_NAME;

// Is the WiFi connected?
boolean gWifiConnected = false;