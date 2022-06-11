#include "furball.h"

#include <Arduino.h>
#include <time.h>

#include <Wire.h>

#include "config.h"
#include "hw.h"

#include <multiball/app.h>
#include <multiball/homebus.h>
#include <multiball/wifi.h>

#ifdef RGB_LED_PIN
#include <Adafruit_NeoPixel.h>
#endif

#ifdef USE_DIAGNOSTICS
#include <diagnostics.h>
#endif

#include "sensors/bme680_sensor.h"
#include "sensors/scd4x_sensor.h"

// #define LUCACE_CCS
#define ADAFRUIT_CCS

#ifdef ADAFRUIT_CCS
#include <Adafruit_CCS811.h>
#endif

#ifdef LUCACE_CCS
#include <CCS811.h>
#endif

#include "sensors/uptime.h"

Uptime uptime;

BME680_Sensor bme680;
SCD4x_Sensor scd4x;
static boolean scd4x_is_present = false;

#ifdef ADAFRUIT_CCS
Adafruit_CCS811 ccs;
#endif

#ifdef LUCACE_CCS
CCS811 ccs;
#endif

static boolean ccs811_is_present = false;

#ifdef RGB_LED_PIN
static Adafruit_NeoPixel pixels(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

typedef enum {
  FURBALL_WORKING = 0, 
  FURBALL_NO_SENSORS = 1,
  FURBALL_NO_WIFI = 2,
  FURBALL_HOMEBUS_FAIL = 3,
  FURBALL_HOMEBUS_WAIT = 4,
  FURBALL_HOMEBUS_MQTT_FAIL = 5
} furball_state_t;

static furball_state_t furball_state;

static furball_state_t furball_get_state() {
  if(!scd4x_is_present)
    return FURBALL_NO_SENSORS;

  if(WiFi.status() != WL_CONNECTED)
    return FURBALL_NO_WIFI;

  homebus_state_t hstate = homebus_state();
  switch(hstate) {
  case HOMEBUS_STATE_PROVISION_WAIT:
    return FURBALL_HOMEBUS_WAIT;
  case HOMEBUS_STATE_SUCCESS:
    return FURBALL_WORKING;
  case HOMEBUS_STATE_UPDATE_CREDENTIALS:
    return FURBALL_HOMEBUS_MQTT_FAIL;
  }

  return FURBALL_HOMEBUS_FAIL;
}

/*
  FURBALL_WORKING - slowly breathing green
  FURBALL_NO_SENSORS - blinking red
  FURBALL_NO_WIFI    - blinking orange
  FURBALL_HOMEBUS_FAIL - rapidly blinking purple
  FURBALL_HOMEBUS_WAIT - breathing purple
  FURBALL_HOMEBUS_MQTT_FAIL - slowly blinking purple
*/
static void led_handle() {
  static time_t next_update = 0;
  static int8_t brightness_delta = 1;

  uint8_t brightness = pixels.getBrightness();
  uint32_t color;

  if(millis() < next_update)
    return;

  /*
  Serial.println("led_handle");
  Serial.println("furball state");
  Serial.println(furball_state);
  Serial.println("brightness");
  Serial.println(brightness);
  */

  switch(furball_state) {
  case FURBALL_WORKING:
    color = pixels.Color(0, 0xff, 0x00);

    if(brightness > 244)
      brightness_delta = -10;
    else if(brightness < 10)
      brightness_delta = 10;

    brightness += brightness_delta;
    next_update = millis() + 100;
    break;

  case FURBALL_NO_SENSORS:
    color = pixels.Color(0xff, 0, 0);
    if(brightness == 0xff)
      brightness = 0;
    else
      brightness = 0xff;

    next_update = millis() + 1000;
    break;

  case FURBALL_NO_WIFI:
    color = pixels.Color(0xff, 0xa5, 0);
    if(brightness == 0xff)
      brightness = 0;
    else
      brightness = 0xff;

    next_update = millis() + 1000;
    break;
  }

  pixels.setBrightness(brightness);
  pixels.setPixelColor(0, color);
  pixels.show();
  return;
  
  pixels.setBrightness(brightness);
  pixels.setPixelColor(0, color);
  pixels.show();
}

void furball_setup() {
  //    Wire.begin(23, 22);
  Wire.begin(8, 9);
  
  delay(1);

  pixels.begin();

  bme680.begin();
  if(bme680.is_present())
    Serial.println("[bme680]");
  else
    Serial.println("BME680 not found");

  delay(1);

  scd4x.begin();
  if(scd4x.is_present()) {
    scd4x_is_present = true;
    Serial.println("[scd4x]");
  }
  else {
    scd4x_is_present = false;
    Serial.println("SCD4x not found");
  }

#ifdef ADAFRUIT_CCS
  if(ccs.begin(0x5a) || ccs.begin(0x5b)) {
    Serial.println("[ccs811]");
    ccs.setDriveMode(CCS811_DRIVE_MODE_10SEC);
    ccs811_is_present = true;
  }
  else
    Serial.println("CCS811 not found");
#endif

#ifdef LUCACE_CCS
  if(ccs.begin(0x5a) || ccs.begin(0x5b)) {
    Serial.println("[ccs811]");
    Serial.println("CCS811 Sensor Enabled:");
    Serial.print("Hardware ID:           0x");
    Serial.println(ccs.getHWID(), HEX);
    Serial.print("Hardware Version:      0x");
    Serial.println(ccs.getHWVersion(), HEX);
    Serial.print("Firmware Boot Version: 0x");
    Serial.println(ccs.getFWBootVersion(), HEX);
    Serial.print("Firmware App Version:  0x");
    Serial.println(ccs.getFWAppVersion(), HEX);
    Serial.println();    

    ccs.setDriveMode(CCS811_DRIVE_MODE_10SEC);
    ccs811_is_present = true;
  }
  else
    Serial.println("CCS811 not found");
#endif

  delay(1);

  homebus_set_provisioner(HOMEBUS_SERVER, HOMEBUS_AUTHENTICATION_TOKEN);

    static const char *publishes[] = {
        DDC_AIR_SENSOR,     DDC_SYSTEM,      DDC_DIAGNOSTIC, DDC_CO2_SENSOR,
        DDC_VOC_SENSOR,     DDC_CCS811_SENSOR, NULL};
    static const char *consumes[] = {NULL};
    static char mac_address[3 * 6];

    strncpy(mac_address, App.mac_address().c_str(), 3 * 6);

    delay(1);

    // this is... wrong - needs to be sorted for proper Homebus use
    homebus_configure("Homebus", "CO2 Monitor", mac_address, "", publishes, consumes);

    homebus_setup();
    Serial.println("[homebus]");

    delay(1);
}


static bool furball_air_update(char *buf, size_t buf_len) {
    if(!bme680.is_present())
        return false;

    snprintf(buf, buf_len, "{ \"temperature\": %.1f, \"humidity\": %.1f, \"pressure\": %.1f }",
#ifdef TEMPERATURE_ADJUSTMENT
             bme680.temperature() + TEMPERATURE_ADJUSTMENT,
#else
             bme680.temperature(),
#endif
             bme680.humidity(), bme680.pressure());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_co2_update(char *buf, size_t buf_len) {
    if(!scd4x.is_present())
        return false;

    snprintf(buf, buf_len,
             "{  \"co2\": %.0f, \"--temperature\": %3.1f, \"--humidity\": "
             "%4.0f, \"--source\": \"SCD4x\" }",
             scd4x.co2(), scd4x.temperature(), scd4x.humidity());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_ccs811_update(char *buf, size_t buf_len) {
  if(!ccs811_is_present)
    return false;

#ifdef ADAFRUIT_CCS
  Serial.print("ccs.readData() -> ");
  int x = ccs.readData();
  Serial.println(x);

  if(x) {
    Serial.println("CCS811 data read failed");
  }

  snprintf(buf, buf_len,
	   "{  \"eco2\": %u, \"tvoc\": %u }",
	   ccs.geteCO2(), ccs.getTVOC());
#endif

#ifdef LUCACE_CCS
    ccs.readStatusRegister();

    if(!ccs.isDATA_READY()) {
      Serial.println("CCS811 not ready");
      if(ccs.hasERROR()) {
	Serial.print("CCS811 error ");
	Serial.println(ccs.getERROR_ID());
      }
      return false;
    }

    snprintf(buf, buf_len,
             "{  \"eco2\": %u, \"tvoc\": %u }",
	     ccs.geteCO2(), ccs.getTVOC());
#endif

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}


/*
 * we do this once at startup, and not again unless our IP address changes
 */
static bool furball_system_update(char *buf, size_t buf_len) {
    static IPAddress oldIP = IPAddress(0, 0, 0, 0);
    static String mac_address = WiFi.macAddress();
    IPAddress local = WiFi.localIP();

    if(oldIP == local)
        return false;

    snprintf(buf, buf_len,
             "{ \"name\": \"%s\", \"platform\": \"%s\", \"build\": \"%s\", "
             "\"ip\": \"%d.%d.%d.%d\", \"mac_addr\": \"%s\" }",
             App.hostname().c_str(), "furball", App.build_info().c_str(), local[0], local[1], local[2], local[3],
             mac_address.c_str());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_diagnostic_update(char *buf, size_t buf_len) {
    snprintf(buf, buf_len,
             "{ \"freeheap\": %d, \"uptime\": %lu, \"rssi\": %d, \"reboots\": "
             "%d, \"wifi_failures\": %d }",
             ESP.getFreeHeap(), uptime.uptime() / 1000, WiFi.RSSI(), App.boot_count(), App.wifi_failures());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

/*
 * LED status:
 *
 * breathing green - all good
 * blinking red - no wifi
 * blinking yellow - no MQTT
 *
 *
 */


void furball_loop() {
    static unsigned long next_loop = 0;

    furball_state = furball_get_state();
    led_handle();

    if(next_loop > millis())
        return;

    next_loop = millis() + UPDATE_INTERVAL;

    Serial.println("bme");
    bme680.handle();
    Serial.println("scd");
    scd4x.handle();

#define BUFFER_LENGTH 700
    char buffer[BUFFER_LENGTH + 1];

    if(furball_air_update(buffer, BUFFER_LENGTH)) {
        homebus_publish_to("org.homebus.experimental.air-sensor", buffer);
    }


    if(furball_co2_update(buffer, BUFFER_LENGTH)) {
        homebus_publish_to("org.homebus.experimental.co2-sensor", buffer);
    }

    if(furball_ccs811_update(buffer, BUFFER_LENGTH)) {
       homebus_publish_to("org.homebus.experimental.ccs811-sensor", buffer);
    }

    if(furball_system_update(buffer, BUFFER_LENGTH)) {
        homebus_publish_to("org.homebus.experimental.system", buffer);
    }

    if(furball_diagnostic_update(buffer, BUFFER_LENGTH)) {
        homebus_publish_to("org.homebus.experimental.diagnostic", buffer);
    }
}

void homebus_mqtt_callback(const char *ddc, const char *payload) {
}

/*
 * this callback is used to stream sensor data for diagnostics
 */
#ifdef USE_DIAGNOSTICS
void furball_stream() {
    static uint8_t count = 0;

    if(count == 0)
      Serial.println("TEMP PRES HUMD TVOC TEMP HUMD CO2");

    if(++count == 10)
        count = 0;

    bme680.handle();
    scd4x.handle();

    Serial.printf("%03.1f %4.0f %4.0f %4.0f %4.0f %03.1f %4.0f\n",
		  bme680.temperature(), bme680.pressure(), bme680.humidity(), bme680.gas_resistance(),
		  scd4x.temperature(), scd4x.humidity(), scd4x.co2());

    if(0) {
        Serial.println("[system]");
        Serial.printf("  Uptime %.2f seconds\n", uptime.uptime() / 1000.0);
        Serial.printf("  Free heap %u bytes\n", ESP.getFreeHeap());
        Serial.printf("  Wifi RSSI %d\n", WiFi.RSSI());
    }
}
#endif
