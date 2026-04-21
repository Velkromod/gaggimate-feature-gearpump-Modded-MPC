#ifndef PUMPTACHOMETER_H
#define PUMPTACHOMETER_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

// Newer ESP-IDF releases expose a dedicated GPIO glitch filter API.
// Keep the include optional so the tach feature still builds on older
// Arduino/ESP-IDF combinations used by this repository.
#if __has_include(<driver/gpio_filter.h>)
#include <driver/gpio_filter.h>
#define PUMP_TACH_HAS_GPIO_FILTER 1
#else
#define PUMP_TACH_HAS_GPIO_FILTER 0
#endif

class PumpTachometer {
  public:
    struct Config {
        uint8_t pin = 0;
        float pulsesPerRevolution = 2.0f;
        uint32_t timeoutUs = 300000;
        uint32_t minPeriodUs = 120;
        // The pump/ESC combination used in this project cannot exceed 5000 RPM.
        // With a two-pulse tach output, that implies a minimum real pulse-to-pulse
        // period of 6000 us. Any shorter interval is treated as physically impossible
        // even if it slips through the hardware glitch filter.
        float maxMechanicalRpm = 5000.0f;
        float emaAlpha = 0.20f;
        bool enableHardwareGlitchFilter = true;
        float maxStepUpRatio = 2.20f;
        float minStepDownRatio = 0.45f;
        // Ignore very short re-triggers after a valid edge to suppress local ringing.
        uint32_t holdoffMinUs = 100;
        uint32_t holdoffMaxUs = 500;
        // Compute a second RPM estimate from hardware-counted pulses over a fixed window.
        uint32_t countWindowUs = 200000;
        // Minimum pulse count required before trusting a new count-window RPM update.
        uint32_t minCountWindowPulses = 6;
        // Period-vs-count-window relative mismatch tolerance before falling back.
        float qualityTolerance = 0.05f;
        // PCNT filter threshold in APB clock cycles (10-bit, max 1023 @ 80 MHz).
        uint16_t pcntFilterCycles = 1023;
        bool enablePcnt = true;
        // Sanity-check PCNT windows against the legacy ISR-accepted pulse count.
        // This catches cases where the hardware counter still overcounts because a
        // single tach pulse produces multiple threshold crossings that survive the
        // short hardware glitch filter.
        float pcntLegacyMinRatio = 0.70f;
        float pcntLegacyMaxRatio = 1.30f;
    };

    struct Sample {
        uint32_t periodUs = 0;
        float rpmInst = 0.0f;
        float rpmEma = 0.0f;
        float rpmCountWindow = 0.0f;
        float rpmPub = 0.0f;
        uint8_t rpmSource = 0;
        bool qualityOk = false;
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
    static constexpr size_t PERIOD_HISTORY_SIZE = 5;

    // The ISR is intentionally tiny: timestamp the edge, reject obviously
    // impossible pulses, then store the accepted period into a short history.
    static void IRAM_ATTR isrThunk(void *arg);
    void IRAM_ATTR onEdgeIsr();

    static uint32_t computeMedianPeriod(const uint32_t *values, size_t count);
    static uint32_t computePhysicalMinPeriodUs(float maxMechanicalRpm, float pulsesPerRevolution);

    Config _config{};
    Sample _sample{};
    bool _enabled = false;

    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    volatile int64_t _lastSeenEdgeUs = 0;
    volatile int64_t _lastAcceptedEdgeUs = 0;
    volatile uint32_t _lastPeriodUs = 0;
    volatile uint32_t _pulseCount = 0;
    volatile uint32_t _glitchRejects = 0;
    volatile uint32_t _periodHistory[PERIOD_HISTORY_SIZE] = {0};
    volatile uint8_t _periodHistoryIndex = 0;
    volatile uint8_t _periodHistoryCount = 0;

    uint32_t _softwareRejects = 0;
    uint32_t _physicalMinPeriodUs = 0;
    uint32_t _lastPublishedPeriodUs = 0;
    int64_t _lastWindowUpdateUs = 0;
    uint32_t _lastWindowPulseCount = 0;

    bool _pcntEnabled = false;
    static constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
    static constexpr pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0;

#if PUMP_TACH_HAS_GPIO_FILTER
    gpio_glitch_filter_handle_t _glitchFilter = nullptr;
#endif
};

#endif // PUMPTACHOMETER_H
