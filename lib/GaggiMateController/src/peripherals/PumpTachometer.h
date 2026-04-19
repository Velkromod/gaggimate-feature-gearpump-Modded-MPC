#ifndef PUMPTACHOMETER_H
#define PUMPTACHOMETER_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

class PumpTachometer {
  public:
    struct Config {
        uint8_t pin = 0;
        float pulsesPerRevolution = 2.0f;
        uint32_t timeoutUs = 300000;
        uint32_t minPeriodUs = 120;
        float emaAlpha = 0.20f;
    };

    struct Sample {
        uint32_t periodUs = 0;
        float rpmInst = 0.0f;
        float rpmEma = 0.0f;
        uint32_t pulseCount = 0;
        uint32_t glitchRejects = 0;
        bool timedOut = true;
    };

    PumpTachometer() = default;

    bool begin(const Config &config);
    void update();
    void reset();

    const Sample &getSample() const { return _sample; }
    bool isEnabled() const { return _enabled; }

  private:
    // The ISR only timestamps edges and stores the minimum amount of data needed
    // to compute RPM later from the task context.
    static void IRAM_ATTR isrThunk(void *arg);
    void IRAM_ATTR onEdgeIsr();

    Config _config{};
    Sample _sample{};
    bool _enabled = false;

    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    volatile int64_t _lastEdgeUs = 0;
    volatile uint32_t _lastPeriodUs = 0;
    volatile uint32_t _pulseCount = 0;
    volatile uint32_t _glitchRejects = 0;
};

#endif // PUMPTACHOMETER_H
