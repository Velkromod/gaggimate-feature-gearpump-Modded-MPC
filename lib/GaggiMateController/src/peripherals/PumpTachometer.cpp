#include "PumpTachometer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

bool PumpTachometer::begin(const Config &config) {
    _config = config;
    _physicalMinPeriodUs = computePhysicalMinPeriodUs(_config.maxMechanicalRpm, _config.pulsesPerRevolution);
    _pcntEnabled = false;
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

    if (_config.enablePcnt) {
        const pcnt_config_t pcntConfig = {
            .pulse_gpio_num = static_cast<int>(_config.pin),
            .ctrl_gpio_num = PCNT_PIN_NOT_USED,
            .lctrl_mode = PCNT_MODE_KEEP,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = PCNT_COUNT_DIS,
            .neg_mode = PCNT_COUNT_INC,
            .counter_h_lim = 32767,
            .counter_l_lim = 0,
            .unit = PCNT_UNIT,
            .channel = PCNT_CHANNEL,
        };

        if (pcnt_unit_config(&pcntConfig) == ESP_OK) {
            const uint16_t filterCycles = static_cast<uint16_t>(std::clamp<uint32_t>(_config.pcntFilterCycles, 0U, 1023U));
            if (filterCycles > 0 && pcnt_set_filter_value(PCNT_UNIT, filterCycles) == ESP_OK) {
                pcnt_filter_enable(PCNT_UNIT);
            }
            pcnt_counter_pause(PCNT_UNIT);
            pcnt_counter_clear(PCNT_UNIT);
            pcnt_counter_resume(PCNT_UNIT);
            _pcntEnabled = true;
        }
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
    _lastSeenEdgeUs = 0;
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
    _lastWindowUpdateUs = 0;
    _lastWindowPulseCount = 0;
    _sample = {};

    if (_pcntEnabled) {
        pcnt_counter_pause(PCNT_UNIT);
        pcnt_counter_clear(PCNT_UNIT);
        pcnt_counter_resume(PCNT_UNIT);
    }
}

void IRAM_ATTR PumpTachometer::isrThunk(void *arg) {
    static_cast<PumpTachometer *>(arg)->onEdgeIsr();
}

void IRAM_ATTR PumpTachometer::onEdgeIsr() {
    const int64_t nowUs = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&_mux);

    uint32_t holdoffUs = _config.holdoffMinUs;
    if (_lastPeriodUs > 0) {
        holdoffUs = std::clamp(_lastPeriodUs / 20U, _config.holdoffMinUs, _config.holdoffMaxUs);
    }

    if (_lastSeenEdgeUs != 0) {
        const uint32_t dtSinceAnyEdgeUs = static_cast<uint32_t>(nowUs - _lastSeenEdgeUs);
        if (holdoffUs > 0 && dtSinceAnyEdgeUs < holdoffUs) {
            // Short re-triggers right after any edge are much more likely to be
            // ringing than real tach transitions.
            _lastSeenEdgeUs = nowUs;
            _glitchRejects++;
            portEXIT_CRITICAL_ISR(&_mux);
            return;
        }
    }

    _lastSeenEdgeUs = nowUs;

    if (_lastAcceptedEdgeUs != 0) {
        const uint32_t dtUs = static_cast<uint32_t>(nowUs - _lastAcceptedEdgeUs);

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

    portENTER_CRITICAL(&_mux);
    lastAcceptedEdgeUs = _lastAcceptedEdgeUs;
    lastPeriodUs = _lastPeriodUs;
    pulseCount = _pulseCount;
    glitchRejects = _glitchRejects;
    periodHistoryCount = _periodHistoryCount;
    for (size_t i = 0; i < PERIOD_HISTORY_SIZE; ++i) {
        periodHistory[i] = _periodHistory[i];
    }
    portEXIT_CRITICAL(&_mux);

    const int64_t nowUs = esp_timer_get_time();

    _sample.pulseCount = pulseCount;
    _sample.glitchRejects = glitchRejects + _softwareRejects;

    if (_lastWindowUpdateUs == 0) {
        _lastWindowUpdateUs = nowUs;
        _lastWindowPulseCount = pulseCount;
        if (_pcntEnabled) {
            pcnt_counter_pause(PCNT_UNIT);
            pcnt_counter_clear(PCNT_UNIT);
            pcnt_counter_resume(PCNT_UNIT);
        }
    } else {
        const int64_t windowElapsedUs = nowUs - _lastWindowUpdateUs;
        if (windowElapsedUs >= static_cast<int64_t>(_config.countWindowUs)) {
            const uint32_t deltaPulsesLegacy = pulseCount - _lastWindowPulseCount;
            uint32_t deltaPulsesPcnt = 0;
            if (_pcntEnabled) {
                int16_t pcntCount = 0;
                pcnt_counter_pause(PCNT_UNIT);
                if (pcnt_get_counter_value(PCNT_UNIT, &pcntCount) == ESP_OK && pcntCount > 0) {
                    deltaPulsesPcnt = static_cast<uint32_t>(pcntCount);
                }
                pcnt_counter_clear(PCNT_UNIT);
                pcnt_counter_resume(PCNT_UNIT);
            }

            uint32_t selectedDeltaPulses = deltaPulsesLegacy;
            if (_pcntEnabled && deltaPulsesPcnt > 0) {
                if (deltaPulsesLegacy >= _config.minCountWindowPulses) {
                    const float ratio = static_cast<float>(deltaPulsesPcnt) / static_cast<float>(std::max<uint32_t>(deltaPulsesLegacy, 1U));
                    if (ratio >= _config.pcntLegacyMinRatio && ratio <= _config.pcntLegacyMaxRatio) {
                        selectedDeltaPulses = deltaPulsesPcnt;
                    } else {
                        selectedDeltaPulses = deltaPulsesLegacy;
                        _softwareRejects++;
                    }
                } else {
                    // If the legacy branch does not have enough pulses yet, allow PCNT to
                    // bootstrap the window on its own.
                    selectedDeltaPulses = deltaPulsesPcnt;
                }
            }

            if (selectedDeltaPulses == 0) {
                _sample.rpmCountWindow = 0.0f;
            } else if (selectedDeltaPulses >= _config.minCountWindowPulses) {
                _sample.rpmCountWindow =
                    (static_cast<float>(selectedDeltaPulses) * 60000000.0f) /
                    (_config.pulsesPerRevolution * static_cast<float>(windowElapsedUs));
            }
            _lastWindowUpdateUs = nowUs;
            _lastWindowPulseCount = pulseCount;
        }
    }

    if (lastAcceptedEdgeUs == 0 || lastPeriodUs == 0 || periodHistoryCount == 0) {
        _sample.periodUs = 0;
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmPub = _sample.rpmCountWindow;
        _sample.rpmEma = (_sample.rpmPub > 0.0f) ? _sample.rpmPub : 0.0f;
        _sample.rpmSource = (_sample.rpmPub > 0.0f) ? 2 : 0;
        _sample.qualityOk = (_sample.rpmPub > 0.0f);
        return;
    }

    if ((nowUs - lastAcceptedEdgeUs) > _config.timeoutUs) {
        // Report zero RPM once the pulse train disappears.
        _sample.periodUs = 0;
        _sample.timedOut = true;
        _sample.rpmInst = 0.0f;
        _sample.rpmCountWindow = 0.0f;
        _sample.rpmPub = 0.0f;
        _sample.rpmEma = 0.0f;
        _sample.rpmSource = 0;
        _sample.qualityOk = false;
        return;
    }

    uint32_t candidatePeriodUs = computeMedianPeriod(periodHistory, periodHistoryCount);

    // Reject abrupt single-update jumps that are much larger than the last
    // reported value. Median filtering already removes isolated spikes, and this
    // plausibility gate catches the remaining occasional missed/extra periods.
    if (_lastPublishedPeriodUs > 0 && candidatePeriodUs > 0) {
        const float ratio = static_cast<float>(candidatePeriodUs) / static_cast<float>(_lastPublishedPeriodUs);
        if (ratio > _config.maxStepUpRatio || ratio < _config.minStepDownRatio) {
            candidatePeriodUs = _lastPublishedPeriodUs;
            _softwareRejects++;
        }
    }

    _lastPublishedPeriodUs = candidatePeriodUs;
    _sample.periodUs = candidatePeriodUs;
    _sample.timedOut = false;
    _sample.rpmInst = 60000000.0f / (_config.pulsesPerRevolution * static_cast<float>(candidatePeriodUs));

    const bool countWindowReady = _sample.rpmCountWindow > 0.0f;
    const float comparisonDenominator = std::max(_sample.rpmCountWindow, 100.0f);
    const float mismatch = countWindowReady ? std::fabs(_sample.rpmInst - _sample.rpmCountWindow) / comparisonDenominator : 0.0f;

    if (countWindowReady && mismatch > _config.qualityTolerance) {
        // Prefer the count-window RPM whenever the instantaneous period branch
        // still disagrees materially with the robust frequency estimate.
        _sample.rpmPub = _sample.rpmCountWindow;
        _sample.rpmSource = 2;
        _sample.qualityOk = false;
    } else {
        _sample.rpmPub = _sample.rpmInst;
        _sample.rpmSource = 1;
        _sample.qualityOk = countWindowReady;
    }

    if (_sample.rpmEma <= 0.0f) {
        _sample.rpmEma = _sample.rpmPub;
    } else {
        // Smooth the published RPM so charts can show a stable trend while the
        // raw instantaneous and count-window estimates remain available.
        _sample.rpmEma += _config.emaAlpha * (_sample.rpmPub - _sample.rpmEma);
    }
}
