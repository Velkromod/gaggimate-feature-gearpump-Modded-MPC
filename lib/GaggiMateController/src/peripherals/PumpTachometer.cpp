#include "PumpTachometer.h"

#include <algorithm>

bool PumpTachometer::begin(const Config &config) {
    _config = config;
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

void PumpTachometer::reset() {
    portENTER_CRITICAL(&_mux);
    _lastEdgeUs = 0;
    _lastPeriodUs = 0;
    _pulseCount = 0;
    _glitchRejects = 0;
    portEXIT_CRITICAL(&_mux);

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

        // Ignore pulses that are too close together to be physically plausible.
        // This provides a lightweight software guard against EMI and ringing.
        if (dtUs < _config.minPeriodUs) {
            _glitchRejects++;
            portEXIT_CRITICAL_ISR(&_mux);
            return;
        }

        _lastPeriodUs = dtUs;
    }

    _lastEdgeUs = nowUs;
    _pulseCount++;

    portEXIT_CRITICAL_ISR(&_mux);
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

    portENTER_CRITICAL(&_mux);
    lastEdgeUs = _lastEdgeUs;
    lastPeriodUs = _lastPeriodUs;
    pulseCount = _pulseCount;
    glitchRejects = _glitchRejects;
    portEXIT_CRITICAL(&_mux);

    _sample.periodUs = lastPeriodUs;
    _sample.pulseCount = pulseCount;
    _sample.glitchRejects = glitchRejects;

    if (lastEdgeUs == 0 || lastPeriodUs == 0) {
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmEma = 0.0f;
        return;
    }

    const int64_t nowUs = esp_timer_get_time();
    if ((nowUs - lastEdgeUs) > _config.timeoutUs) {
        // Report zero RPM once the pulse train disappears.
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmEma = 0.0f;
        return;
    }

    _sample.timedOut = false;
    _sample.rpmInst = 60000000.0f / (_config.pulsesPerRevolution * static_cast<float>(lastPeriodUs));

    if (_sample.rpmEma <= 0.0f) {
        _sample.rpmEma = _sample.rpmInst;
    } else {
        // Keep a lightly filtered RPM for telemetry consumers that prefer a
        // steadier value than the instantaneous period-based estimate.
        _sample.rpmEma += _config.emaAlpha * (_sample.rpmInst - _sample.rpmEma);
    }
}
