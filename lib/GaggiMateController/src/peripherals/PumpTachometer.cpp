#include "PumpTachometer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {
uint32_t captureTicksToUs(uint32_t ticks) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(ticks) * 1000000ULL + 40000000ULL) / 80000000ULL);
}
} // namespace

bool PumpTachometer::begin(const Config &config) {
    _config = config;
    _physicalMinPeriodUs =
        computePhysicalMinPeriodUs(_config.maxMechanicalRpm, _config.pulsesPerRevolution);
    _captureEnabled = false;
    _captureEnabledCfg = _config.enableMcpwmCapture;
    _captureInitOk = false;
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

    if (_config.enableMcpwmCapture) {
        if (mcpwm_gpio_init(MCPWM_UNIT, MCPWM_CAP_0, static_cast<int>(_config.pin)) == ESP_OK) {
            mcpwm_capture_config_t capConfig = {};
            capConfig.cap_edge = MCPWM_NEG_EDGE;
            capConfig.cap_prescale =
                std::clamp<uint32_t>(_config.mcpwmCapturePrescale, 1U, 256U);
            capConfig.capture_cb = &PumpTachometer::captureThunk;
            capConfig.user_data = this;

            if (mcpwm_capture_enable_channel(MCPWM_UNIT, MCPWM_CAPTURE_CHANNEL, &capConfig) ==
                ESP_OK) {
                _captureEnabled = true;
                _captureInitOk = true;
            }
        }
    }

    if (!_captureEnabled) {
        // The ISR service may already be installed by another module. Treat that as
        // success and only fail on unexpected errors.
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            _enabled = false;
            return false;
        }

        gpio_isr_handler_remove(static_cast<gpio_num_t>(_config.pin));
        if (gpio_isr_handler_add(static_cast<gpio_num_t>(_config.pin), &PumpTachometer::isrThunk,
                                 this) != ESP_OK) {
            _enabled = false;
            return false;
        }
    }

    _enabled = true;
    return true;
}

uint32_t PumpTachometer::computePhysicalMinPeriodUs(float maxMechanicalRpm,
                                                    float pulsesPerRevolution) {
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
    _lastAcceptedEdgeUs = 0;
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
    _lastPublishedPeriodUs = 0;
    _lastConfidencePulseCount = 0;
    _lastConfidenceRejectCount = 0;

    _lastCaptureValue = 0;
    _captureActive = false;
    _captureEventCount = 0;
    _captureLastPeriodTicks = 0;
    _captureLastEdge = 0;

    _sample = {};
}

void IRAM_ATTR PumpTachometer::isrThunk(void *arg) {
    static_cast<PumpTachometer *>(arg)->onEdgeIsr();
}

bool IRAM_ATTR PumpTachometer::captureThunk(mcpwm_unit_t,
                                            mcpwm_capture_channel_id_t,
                                            const cap_event_data_t *edata,
                                            void *userData) {
    if (edata == nullptr || userData == nullptr) {
        return false;
    }

    static_cast<PumpTachometer *>(userData)->onCaptureIsr(
        edata->cap_value, static_cast<uint32_t>(edata->cap_edge));
    return false;
}

void IRAM_ATTR PumpTachometer::onEdgeIsr() {
    onCaptureIsr(static_cast<uint32_t>(esp_timer_get_time()), 0U);
}

void IRAM_ATTR PumpTachometer::onCaptureIsr(uint32_t captureValue, uint32_t captureEdge) {
    const int64_t nowUs = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&_mux);

    if (_captureEnabled) {
        _captureActive = true;
        _captureEventCount++;
        _captureLastEdge = captureEdge;
    }

    if (_lastAcceptedEdgeUs != 0) {
        uint32_t dtUs = 0;
        if (_captureEnabled) {
            const uint32_t dtTicks = captureValue - _lastCaptureValue;
            _captureLastPeriodTicks = dtTicks;
            dtUs = captureTicksToUs(dtTicks);
        } else {
            _captureLastPeriodTicks = 0;
            dtUs = static_cast<uint32_t>(nowUs - _lastAcceptedEdgeUs);
        }

        uint32_t holdoffUs = _config.holdoffMinUs;
        if (_lastPeriodUs > 0) {
            const uint32_t divisor =
                std::max<uint32_t>(static_cast<uint32_t>(_config.holdoffPeriodDivisor), 1U);
            holdoffUs = std::clamp(_lastPeriodUs / divisor, _config.holdoffMinUs,
                                   _config.holdoffMaxUs);
        }

        if (holdoffUs > 0 && dtUs < holdoffUs) {
            _glitchRejects++;
            portEXIT_CRITICAL_ISR(&_mux);
            return;
        }

        const uint32_t minAcceptedPeriodUs =
            std::max(_config.minPeriodUs, _physicalMinPeriodUs);
        if (minAcceptedPeriodUs > 0 && dtUs < minAcceptedPeriodUs) {
            _glitchRejects++;
            portEXIT_CRITICAL_ISR(&_mux);
            return;
        }

        _lastPeriodUs = dtUs;
        _periodHistory[_periodHistoryIndex] = dtUs;
        _periodHistoryIndex =
            static_cast<uint8_t>((_periodHistoryIndex + 1) % PERIOD_HISTORY_SIZE);
        if (_periodHistoryCount < PERIOD_HISTORY_SIZE) {
            _periodHistoryCount++;
        }
    }

    _lastCaptureValue = captureValue;
    _lastAcceptedEdgeUs = nowUs;
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

    int64_t lastAcceptedEdgeUs = 0;
    uint32_t lastPeriodUs = 0;
    uint32_t pulseCount = 0;
    uint32_t glitchRejects = 0;
    uint32_t periodHistory[PERIOD_HISTORY_SIZE] = {0};
    uint8_t periodHistoryCount = 0;
    bool captureActive = false;
    uint32_t captureEventCount = 0;
    uint32_t captureLastPeriodTicks = 0;
    uint32_t captureLastEdge = 0;

    portENTER_CRITICAL(&_mux);
    lastAcceptedEdgeUs = _lastAcceptedEdgeUs;
    lastPeriodUs = _lastPeriodUs;
    pulseCount = _pulseCount;
    glitchRejects = _glitchRejects;
    periodHistoryCount = _periodHistoryCount;
    captureActive = _captureActive;
    captureEventCount = _captureEventCount;
    captureLastPeriodTicks = _captureLastPeriodTicks;
    captureLastEdge = _captureLastEdge;
    for (size_t i = 0; i < PERIOD_HISTORY_SIZE; ++i) {
        periodHistory[i] = _periodHistory[i];
    }
    portEXIT_CRITICAL(&_mux);

    const int64_t nowUs = esp_timer_get_time();

    // Deprecated compatibility placeholders after removing PCNT from the RPM path.
    _sample.rpmCountWindow = 0.0f;
    _sample.rpmCountWindowPcntRaw = 0.0f;
    _sample.pcntRatio = 0.0f;
    _sample.pcntHealthy = false;

    _sample.pulseCount = pulseCount;
    _sample.glitchRejects = glitchRejects;
    _sample.captureEnabledCfg = _captureEnabledCfg;
    _sample.captureInitOk = _captureInitOk;
    _sample.captureActive = captureActive;
    _sample.captureEventCount = captureEventCount;
    _sample.captureLastPeriodUs =
        _captureEnabled ? captureTicksToUs(captureLastPeriodTicks) : 0U;
    _sample.captureLastEdge = captureLastEdge;

    if (lastAcceptedEdgeUs == 0 || lastPeriodUs == 0 || periodHistoryCount == 0) {
        _sample.periodUs = 0;
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmPub = 0.0f;
        _sample.rpmEma = 0.0f;
        _sample.rpmSource = 0;
        _sample.qualityOk = false;
        _sample.rpmValid = false;
        _sample.rpmConfidence = 0.0f;
        return;
    }

    if ((nowUs - lastAcceptedEdgeUs) > _config.timeoutUs) {
        _sample.periodUs = 0;
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmPub = 0.0f;
        _sample.rpmEma = 0.0f;
        _sample.rpmSource = 0;
        _sample.qualityOk = false;
        _sample.rpmValid = false;
        _sample.rpmConfidence = 0.0f;
        return;
    }

    uint32_t candidatePeriodUs = computeMedianPeriod(periodHistory, periodHistoryCount);
    bool plausibilityRejected = false;

    // Reject abrupt single-update jumps that are much larger than the last
    // reported value. Median filtering already removes isolated spikes, and this
    // plausibility gate catches the remaining occasional missed/extra periods.
    if (_lastPublishedPeriodUs > 0 && candidatePeriodUs > 0) {
        const float ratio =
            static_cast<float>(candidatePeriodUs) /
            static_cast<float>(_lastPublishedPeriodUs);
        if (ratio > _config.maxStepUpRatio || ratio < _config.minStepDownRatio) {
            candidatePeriodUs = _lastPublishedPeriodUs;
            _softwareRejects++;
            plausibilityRejected = true;
        }
    }

    _lastPublishedPeriodUs = candidatePeriodUs;
    _sample.periodUs = candidatePeriodUs;
    _sample.timedOut = false;
    _sample.rpmInst =
        60000000.0f / (_config.pulsesPerRevolution * static_cast<float>(candidatePeriodUs));
    _sample.rpmPub = _sample.rpmInst;
    _sample.rpmSource = _captureEnabled ? 1 : 2;

    const uint32_t totalRejects = glitchRejects + _softwareRejects;
    const uint32_t deltaAccepted = pulseCount - _lastConfidencePulseCount;
    const uint32_t deltaRejects = totalRejects - _lastConfidenceRejectCount;
    _lastConfidencePulseCount = pulseCount;
    _lastConfidenceRejectCount = totalRejects;

    float recentAcceptRatio = 1.0f;
    const uint32_t deltaEvents = deltaAccepted + deltaRejects;
    if (deltaEvents > 0) {
        recentAcceptRatio =
            static_cast<float>(deltaAccepted) / static_cast<float>(deltaEvents);
    }

    float confidence = _captureEnabled ? 1.0f : 0.70f;
    if (periodHistoryCount < PERIOD_HISTORY_SIZE) {
        confidence *= 0.50f;
    }
    confidence *= std::clamp(recentAcceptRatio, 0.0f, 1.0f);
    if (plausibilityRejected) {
        confidence *= 0.65f;
    }

    const float ageNorm =
        std::clamp(static_cast<float>(nowUs - lastAcceptedEdgeUs) /
                       static_cast<float>(std::max<uint32_t>(_config.timeoutUs, 1U)),
                   0.0f, 1.0f);
    confidence *= (1.0f - 0.50f * ageNorm);

    _sample.rpmConfidence = std::clamp(confidence, 0.0f, 1.0f);
    _sample.rpmValid =
        (periodHistoryCount >= PERIOD_HISTORY_SIZE) &&
        (_sample.periodUs > 0) &&
        (_sample.rpmConfidence >= 0.35f);
    _sample.qualityOk = _sample.rpmValid;
    _sample.glitchRejects = totalRejects;

    if (_sample.rpmEma <= 0.0f) {
        _sample.rpmEma = _sample.rpmPub;
    } else {
        // Smooth the published RPM so charts can show a stable trend while the
        // raw instantaneous estimate remains available.
        _sample.rpmEma += _config.emaAlpha * (_sample.rpmPub - _sample.rpmEma);
    }
}