#ifndef PUMPTACHOMETER_H
#define PUMPTACHOMETER_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
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
        // period of 6000 us. Any shorter interval is treated as physically
        // impossible even if it slips through the hardware glitch filter.
        float maxMechanicalRpm = 5000.0f;

        float emaAlpha = 0.20f;
        bool enableHardwareGlitchFilter = true;

        // Plausibility gate relative to the last published period.
        float maxStepUpRatio = 2.20f;
        float minStepDownRatio = 0.45f;

        // Holdoff is anchored to the last accepted edge, not the last seen edge.
        // Use a dynamic blanking window derived from the last accepted period.
        uint32_t holdoffMinUs = 250;
        uint32_t holdoffMaxUs = 8000;
        uint8_t holdoffPeriodDivisor = 2;

        bool enableMcpwmCapture = true;
        uint32_t mcpwmCapturePrescale = 1;
    };

    struct Sample {
        uint32_t periodUs = 0;
        float rpmInst = 0.0f;
        float rpmEma = 0.0f;

        // Deprecated compatibility placeholders kept to avoid breaking
        // existing telemetry tooling immediately after removing PCNT.
        float rpmCountWindow = 0.0f;
        float rpmCountWindowPcntRaw = 0.0f;
        float pcntRatio = 0.0f;

        float rpmPub = 0.0f;

        // 0 = none, 1 = MCPWM capture path, 2 = GPIO/esp_timer fallback path
        uint8_t rpmSource = 0;

        // Legacy alias kept for compatibility with existing telemetry consumers.
        bool qualityOk = false;

        // New fields intended for estimator use.
        bool rpmValid = false;
        float rpmConfidence = 0.0f;

        bool captureEnabledCfg = false;
        bool captureInitOk = false;
        bool captureActive = false;
        uint32_t captureEventCount = 0;
        uint32_t captureLastPeriodUs = 0;
        uint32_t captureLastEdge = 0;

        // Deprecated compatibility placeholder after removing PCNT.
        bool pcntHealthy = false;

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
    static constexpr size_t PERIOD_HISTORY_SIZE = 3;

    // The ISR is intentionally tiny: timestamp the edge, reject obviously
    // impossible pulses, then store the accepted period into a short history.
    static void IRAM_ATTR isrThunk(void *arg);
    static bool IRAM_ATTR captureThunk(mcpwm_unit_t mcpwm,
                                       mcpwm_capture_channel_id_t capChannel,
                                       const cap_event_data_t *edata,
                                       void *userData);
    void IRAM_ATTR onEdgeIsr();
    void IRAM_ATTR onCaptureIsr(uint32_t captureValue, uint32_t captureEdge);

    static uint32_t computeMedianPeriod(const uint32_t *values, size_t count);
    static uint32_t computePhysicalMinPeriodUs(float maxMechanicalRpm,
                                               float pulsesPerRevolution);

    Config _config{};
    Sample _sample{};
    bool _enabled = false;

    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

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

    // Recent confidence bookkeeping.
    uint32_t _lastConfidencePulseCount = 0;
    uint32_t _lastConfidenceRejectCount = 0;

    bool _captureEnabled = false;
    bool _captureEnabledCfg = false;
    bool _captureInitOk = false;
    volatile bool _captureActive = false;
    volatile uint32_t _captureEventCount = 0;
    volatile uint32_t _captureLastPeriodTicks = 0;
    volatile uint32_t _captureLastEdge = 0;

    static constexpr mcpwm_unit_t MCPWM_UNIT = MCPWM_UNIT_0;
    static constexpr mcpwm_capture_channel_id_t MCPWM_CAPTURE_CHANNEL = MCPWM_SELECT_CAP0;
    static constexpr uint32_t MCPWM_CAPTURE_HZ = 80000000UL;

    volatile uint32_t _lastCaptureValue = 0;

#if PUMP_TACH_HAS_GPIO_FILTER
    gpio_glitch_filter_handle_t _glitchFilter = nullptr;
#endif
};

#endif // PUMPTACHOMETER_H