#include <Arduino.h>

#include "config.h"
#include "hw.h"

#include <multiball/app.h>
#include <multiball/homebus.h>
#include <multiball/ota_updates.h>
#include <multiball/wifi.h>

#ifdef USE_DIAGNOSTICS
#include <diagnostics.h>
#endif

#include <ESPmDNS.h>

#include <FS.h>
#include <SPIFFS.h>

#include "furball.h"

#define APP_NAME "furball"

MultiballApp App;

void setup() {
    const wifi_credential_t wifi_credentials[] = {
        {WIFI_SSID1, WIFI_PASSWORD1}, {WIFI_SSID2, WIFI_PASSWORD2}, {WIFI_SSID3, WIFI_PASSWORD3}};

    Serial.begin(115200);

    Serial.println("wifis:");
    Serial.printf("  %32s %32s\n", WIFI_SSID1, WIFI_PASSWORD1);
    Serial.printf("  %32s %32s\n", WIFI_SSID2, WIFI_PASSWORD2);
    Serial.printf("  %32s %32s\n", WIFI_SSID3, WIFI_PASSWORD3);

    delay(500);

#ifdef USE_DIAGNOSTICS
    diagnostics_setup(APP_NAME);
    Serial.println("[diagnostics]");
#endif

    delay(500);

    App.wifi_credentials(3, wifi_credentials);
    App.begin(APP_NAME);
    Serial.println("[app]");

    furball_setup();
    Serial.println("[furball]");


    delay(500);
}

void loop() {
#ifdef USE_DIAGNOSTICS
    diagnostics_loop(furball_stream);
#endif

    App.handle();

    furball_loop();
}
