#pragma once

#include "sensor.h"

#include <Sparkfun_SCD4x_Arduino_Library.h>

class SCD4x_Sensor : public Sensor {
  public:
    void begin();
    void handle();

    float temperature() {
        _mark_read();
        return _temperature;
    };
    float humidity() {
        _mark_read();
        return _humidity;
    };
    float co2() {
        _mark_read();
        return _co2;
    };

  private:
    SCD4x _scd4xx = SCD4x(SCD4x_SENSOR_SCD41);

    float _temperature;
    float _humidity;
    float _co2;
};
