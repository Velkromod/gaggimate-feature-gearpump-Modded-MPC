#include "PumpTachometer.h"

#include <algorithm>
#include <cmath>

bool PumpTachometer::begin(const Config &config) {
    _config = config;
    _physicalMinPeriodUs = computePhysicalMinPeriodUs(_config.maxMechanicalRpm, _config.pulsesPerRevolution);
    reset();

    if (_config.pin == 0) {
        _enabled = false;
        return false;
    }

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << _config.pin);
    io.mode = GPIO_MODE_INPUT;
    // The tach signal already arrives through an external voltage divider, so the
    // GPIO should not enable any internal pull resistors.
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_NEGEDGE;

    if (gpio_config(&io) != ESP_OK) {
        _enabled = false;
        return false;
    }

#if PUMP_TACH_HAS_GPIO_FILTER
    if (_config.enableHardwareGlitchFilter) {
        gpio_pin_glitch_filter_config_t filterConfig = {
            .gpio_num = static_cast<gpio_num_t>(_config.pin),
        };

        if (gpio_new_pin_glitch_filter(&filterConfig, &_glitchFilter) == ESP_OK) {
            gpio_glitch_filter_enable(_glitchFilter);
        }
    }
#endif

    // The ISR service may already be installed by another module. Treat that as
    // success and only fail on unexpected errors.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        _enabled = false;
        return false;
    }

    gpio_isr_handler_remove(static_cast<gpio_num_t>(_config.pin));
    if (gpio_isr_handler_add(static_cast<gpio_num_t>(_config.pin), &PumpTachometer::isrThunk, this) != ESP_OK) {
        _enabled = false;
        return false;
    }

    _enabled = true;
    return true;
}

uint32_t PumpTachometer::computePhysicalMinPeriodUs(float maxMechanicalRpm, float pulsesPerRevolution) {
    if (!(maxMechanicalRpm > 0.0f) || !(pulsesPerRevolution > 0.0f)) {
        return 0;
    }

    const float pulsesPerSecond = (maxMechanicalRpm / 60.0f) * pulsesPerRevolution;
    if (!(pulsesPerSecond > 0.0f)) {
        return 0;
    }

    return static_cast<uint32_t>(std::lround(1000000.0f / pulsesPerSecond));
}

void PumpTachometer::reset() {
    portENTER_CRITICAL(&_mux);
    _lastEdgeUs = 0;
    _lastPeriodUs = 0;
    _pulseCount = 0;
    _glitchRejects = 0;
    _periodHistoryIndex = 0;
    _periodHistoryCount = 0;
    for (size_t i = 0; i < PERIOD_HISTORY_SIZE; ++i) {
        _periodHistory[i] = 0;
    }
    portEXIT_CRITICAL(&_mux);

    _softwareRejects = 0;
    _sample = {};
}

void IRAM_ATTR PumpTachometer::isrThunk(void *arg) {
    static_cast<PumpTachometer *>(arg)->onEdgeIsr();
}

void IRAM_ATTR PumpTachometer::onEdgeIsr() {
    const int64_t nowUs = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&_mux);

    if (_lastEdgeUs != 0) {
        const uint32_t dtUs = static_cast<uint32_t>(nowUs - _lastEdgeUs);

        // Reject edges that are closer together than either the generic EMI floor
        // or the physical floor implied by the maximum mechanical RPM. This keeps
        // the tach reader from accepting impossible sub-periods when a single
        // electrical pulse rings or crosses the input threshold multiple times.
        const uint32_t minAcceptedPeriodUs = std::max(_config.minPeriodUs, _physicalMinPeriodUs);
        if (minAcceptedPeriodUs > 0 && dtUs < minAcceptedPeriodUs) {
            _glitchRejects++;
            portEXIT_CRITICAL_ISR(&_mux);
            return;
        }

        _lastPeriodUs = dtUs;
        _periodHistory[_periodHistoryIndex] = dtUs;
        _periodHistoryIndex = static_cast<uint8_t>((_periodHistoryIndex + 1) % PERIOD_HISTORY_SIZE);
        if (_periodHistoryCount < PERIOD_HISTORY_SIZE) {
            _periodHistoryCount++;
        }
    }

    _lastEdgeUs = nowUs;
    _pulseCount++;

    portEXIT_CRITICAL_ISR(&_mux);
}

uint32_t PumpTachometer::computeMedianPeriod(const uint32_t *values, size_t count) {
    if (count == 0) {
        return 0;
    }

    uint32_t scratch[PERIOD_HISTORY_SIZE] = {0};
    for (size_t i = 0; i < count; ++i) {
        scratch[i] = values[i];
    }

    std::sort(scratch, scratch + count);
    return scratch[count / 2];
}

void PumpTachometer::update() {
    if (!_enabled) {
        _sample = {};
        return;
    }

    int64_t lastEdgeUs = 0;
    uint32_t lastPeriodUs = 0;
    uint32_t pulseCount = 0;
    uint32_t glitchRejects = 0;
    uint32_t periodHistory[PERIOD_HISTORY_SIZE] = {0};
    uint8_t periodHistoryCount = 0;

    portENTER_CRITICAL(&_mux);
    lastEdgeUs = _lastEdgeUs;
    lastPeriodUs = _lastPeriodUs;
    pulseCount = _pulseCount;
    glitchRejects = _glitchRejects;
    periodHistoryCount = _periodHistoryCount;
    for (size_t i = 0; i < PERIOD_HISTORY_SIZE; ++i) {
        periodHistory[i] = _periodHistory[i];
    }
    portEXIT_CRITICAL(&_mux);

    _sample.pulseCount = pulseCount;
    _sample.glitchRejects = glitchRejects + _softwareRejects;

    if (lastEdgeUs == 0 || lastPeriodUs == 0 || periodHistoryCount == 0) {
        _sample.periodUs = 0;
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmEma = 0.0f;
        return;
    }

    const int64_t nowUs = esp_timer_get_time();
    if ((nowUs - lastEdgeUs) > _config.timeoutUs) {
        // Report zero RPM once the pulse train disappears.
        _sample.periodUs = 0;
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmEma = 0.0f;
        return;
    }

    uint32_t candidatePeriodUs = computeMedianPeriod(periodHistory, periodHistoryCount);

    // Reject abrupt single-update jumps that are much larger than the last
    // reported value. Median filtering already removes isolated spikes, and this
    // plausibility gate catches the remaining occasional missed/extra periods.
    if (_sample.periodUs > 0 && candidatePeriodUs > 0) {
        const float ratio = static_cast<float>(candidatePeriodUs) / static_cast<float>(_sample.periodUs);
        if (ratio > _config.maxStepUpRatio || ratio < _config.minStepDownRatio) {
            candidatePeriodUs = _sample.periodUs;
            _softwareRejects++;
        }
    }

    _sample.periodUs = candidatePeriodUs;
    _sample.timedOut = false;
    _sample.rpmInst = 60000000.0f / (_config.pulsesPerRevolution * static_cast<float>(candidatePeriodUs));

    if (_sample.rpmEma <= 0.0f) {
        _sample.rpmEma = _sample.rpmInst;
    } else {
        // Keep a lightly filtered RPM for telemetry consumers that prefer a
        // steadier value than the instantaneous period-based estimate.
        _sample.rpmEma += _config.emaAlpha * (_sample.rpmInst - _sample.rpmEma);
    }
}
