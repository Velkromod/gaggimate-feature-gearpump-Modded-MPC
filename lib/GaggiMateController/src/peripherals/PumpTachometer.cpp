#include "PumpTachometer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <esp_timer.h>

namespace {

uint32_t captureTicksToUs(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000000ULL + 40000000ULL) /
                                 80000000ULL);
}

uint32_t computeHoldoffUs(const PumpTachometer::Config &config,
                         uint32_t lastPeriodUs) {
    if (lastPeriodUs == 0) {
        return config.holdoffMinUs;
    }

    const uint32_t divisor =
        std::max<uint32_t>(static_cast<uint32_t>(config.holdoffPeriodDivisor), 1U);
    return std::clamp(lastPeriodUs / divisor, config.holdoffMinUs,
                      config.holdoffMaxUs);
}

} // namespace

bool PumpTachometer::begin(const Config &config) {
    _config = config;
    _physicalMinPeriodUs =
        computePhysicalMinPeriodUs(_config.maxMechanicalRpm, _config.pulsesPerRevolution);
    _pcntEnabled = false;
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
            const uint16_t filterCycles =
                static_cast<uint16_t>(std::clamp<uint32_t>(_config.pcntFilterCycles, 0U, 1023U));
            if (filterCycles > 0 &&
                pcnt_set_filter_value(PCNT_UNIT, filterCycles) == ESP_OK) {
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

    if (_config.enableMcpwmCapture) {
        if (mcpwm_gpio_init(MCPWM_UNIT, MCPWM_CAP_0, static_cast<int>(_config.pin)) ==
            ESP_OK) {
            mcpwm_capture_config_t capConfig = {};
            capConfig.cap_edge = MCPWM_NEG_EDGE;
            capConfig.cap_prescale =
                std::clamp<uint32_t>(_config.mcpwmCapturePrescale, 1U, 256U);
            capConfig.capture_cb = &PumpTachometer::captureThunk;
            capConfig.user_data = this;

            if (mcpwm_capture_enable_channel(MCPWM_UNIT, MCPWM_CAPTURE_CHANNEL,
                                             &capConfig) == ESP_OK) {
                _captureEnabled = true;
                _captureInitOk = true;
            }
        }
    }

    // Exactly one edge source must feed onCaptureIsr() at a time:
    //  - MCPWM capture callback when capture init succeeds.
    //  - GPIO ISR only as fallback when capture is unavailable.
    const esp_err_t isrInstallErr = gpio_install_isr_service(0);
    if (isrInstallErr != ESP_OK && isrInstallErr != ESP_ERR_INVALID_STATE) {
        _enabled = false;
        return false;
    }

    gpio_isr_handler_remove(static_cast<gpio_num_t>(_config.pin));
    if (!_captureEnabled) {
        if (gpio_install_isr_service(0) != ESP_OK &&
            gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK) {
            _enabled = false;
            return false;
        }

        if (gpio_isr_handler_add(static_cast<gpio_num_t>(_config.pin),
                                 &PumpTachometer::isrThunk, this) != ESP_OK) {
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
    _pcntHealthy = false;
    _pcntGoodWindows = 0;
    _pcntBadWindows = 0;
    _captureHealthy = false;
    _captureGoodWindows = 0;
    _captureBadWindows = 0;
    _lastCaptureValue = 0;
    _captureActive = false;
    _captureEventCount = 0;
    _captureLastPeriodTicks = 0;
    _captureLastEdge = 0;
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

void IRAM_ATTR PumpTachometer::onCaptureIsr(uint32_t captureValue,
                                            uint32_t captureEdge) {
    const int64_t nowUs = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&_mux);

    // Telemetry only: this timestamp must not drive acceptance gates.
    _lastSeenEdgeUs = nowUs;

    _captureActive = _captureEnabled;
    if (_captureEnabled) {
        _captureEventCount++;
        _captureLastEdge = captureEdge;
    } else {
        _captureLastEdge = 0;
    }

    const int64_t lastAcceptedEdgeUs = _lastAcceptedEdgeUs;
    const uint32_t holdoffUs = computeHoldoffUs(_config, _lastPeriodUs);

    if (lastAcceptedEdgeUs != 0 && nowUs > lastAcceptedEdgeUs) {
        const uint32_t dtSinceAcceptedEdgeUs =
            static_cast<uint32_t>(nowUs - lastAcceptedEdgeUs);
        if (holdoffUs > 0 && dtSinceAcceptedEdgeUs < holdoffUs) {
            // Reject burst-noise edges without advancing any blanking reference.
            _glitchRejects++;
            portEXIT_CRITICAL_ISR(&_mux);
            return;
        }
    }

    if (lastAcceptedEdgeUs != 0) {
        uint32_t dtUs = 0;
        uint32_t dtTicks = 0;
        if (_captureEnabled) {
            dtTicks = captureValue - _lastCaptureValue;
            dtUs = captureTicksToUs(dtTicks);
        } else {
            dtUs = static_cast<uint32_t>(nowUs - lastAcceptedEdgeUs);
        }

        const uint32_t minAcceptedPeriodUs =
            std::max(_config.minPeriodUs, _physicalMinPeriodUs);
        if (minAcceptedPeriodUs > 0 && dtUs < minAcceptedPeriodUs) {
            _glitchRejects++;
            portEXIT_CRITICAL_ISR(&_mux);
            return;
        }

        if (_lastPeriodUs > 0 && dtUs > 0) {
            const float ratio =
                static_cast<float>(dtUs) / static_cast<float>(_lastPeriodUs);
            if (ratio > _config.maxStepUpRatio ||
                ratio < _config.minStepDownRatio) {
                _glitchRejects++;
                portEXIT_CRITICAL_ISR(&_mux);
                return;
            }
        }

        _lastPeriodUs = dtUs;
        if (_captureEnabled) {
            _captureLastPeriodTicks = dtTicks;
        }
        _periodHistory[_periodHistoryIndex] = dtUs;
        _periodHistoryIndex =
            static_cast<uint8_t>((_periodHistoryIndex + 1) % PERIOD_HISTORY_SIZE);
        if (_periodHistoryCount < PERIOD_HISTORY_SIZE) {
            _periodHistoryCount++;
        }
    }

    // Update pulse/period state only after acceptance.
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
    _sample.captureEnabledCfg = _captureEnabledCfg;
    _sample.captureInitOk = _captureInitOk;
    _sample.captureActive = _captureActive;
    _sample.captureEventCount = _captureEventCount;
    _sample.captureLastPeriodUs = captureTicksToUs(_captureLastPeriodTicks);
    _sample.captureLastEdge = _captureLastEdge;

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
            const uint32_t deltaPulsesAccepted = pulseCount - _lastWindowPulseCount;
            uint32_t deltaPulsesPcntRaw = 0;
            if (_pcntEnabled) {
                int16_t pcntCount = 0;
                pcnt_counter_pause(PCNT_UNIT);
                if (pcnt_get_counter_value(PCNT_UNIT, &pcntCount) == ESP_OK &&
                    pcntCount > 0) {
                    deltaPulsesPcntRaw = static_cast<uint32_t>(pcntCount);
                }
                pcnt_counter_clear(PCNT_UNIT);
                pcnt_counter_resume(PCNT_UNIT);
            }

            if (deltaPulsesAccepted >= _config.minCountWindowPulses) {
                _sample.rpmCountWindow =
                    (static_cast<float>(deltaPulsesAccepted) * 60000000.0f) /
                    (_config.pulsesPerRevolution *
                     static_cast<float>(windowElapsedUs));
            } else if (deltaPulsesAccepted == 0) {
                _sample.rpmCountWindow = 0.0f;
            }

            if (deltaPulsesPcntRaw >= _config.minCountWindowPulses) {
                _sample.rpmCountWindowPcntRaw =
                    (static_cast<float>(deltaPulsesPcntRaw) * 60000000.0f) /
                    (_config.pulsesPerRevolution *
                     static_cast<float>(windowElapsedUs));
            } else if (deltaPulsesPcntRaw == 0) {
                _sample.rpmCountWindowPcntRaw = 0.0f;
            }

            _sample.pcntRatio = 0.0f;
            bool pcntWindowHealthy = false;
            if (_pcntEnabled &&
                deltaPulsesAccepted >= _config.minCountWindowPulses &&
                deltaPulsesPcntRaw > 0) {
                _sample.pcntRatio =
                    static_cast<float>(deltaPulsesPcntRaw) /
                    static_cast<float>(std::max<uint32_t>(deltaPulsesAccepted, 1U));
                pcntWindowHealthy =
                    (_sample.pcntRatio >= _config.pcntLegacyMinRatio) &&
                    (_sample.pcntRatio <= _config.pcntLegacyMaxRatio);
            }

            if (_pcntEnabled) {
                if (pcntWindowHealthy) {
                    _pcntGoodWindows =
                        static_cast<uint8_t>(std::min<uint32_t>(_pcntGoodWindows + 1U,
                                                               255U));
                    _pcntBadWindows = 0;
                    if (_pcntGoodWindows >= _config.pcntHealthyRequireGoodWindows) {
                        _pcntHealthy = true;
                    }
                } else {
                    if (deltaPulsesPcntRaw > 0 &&
                        deltaPulsesAccepted >= _config.minCountWindowPulses) {
                        _softwareRejects++;
                    }
                    _pcntBadWindows =
                        static_cast<uint8_t>(std::min<uint32_t>(_pcntBadWindows + 1U,
                                                               255U));
                    _pcntGoodWindows = 0;
                    if (_pcntBadWindows >= _config.pcntHealthyRequireBadWindows) {
                        _pcntHealthy = false;
                    }
                }
            } else {
                _pcntHealthy = false;
            }
            _sample.pcntHealthy = _pcntHealthy;

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
        _sample.pcntHealthy = _pcntHealthy;
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
        _sample.rpmCountWindowPcntRaw = 0.0f;
        _sample.pcntRatio = 0.0f;
        _sample.rpmSource = 0;
        _sample.qualityOk = false;
        _sample.pcntHealthy = _pcntHealthy;
        return;
    }

    uint32_t candidatePeriodUs = computeMedianPeriod(periodHistory, periodHistoryCount);

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
        }
    }

    _lastPublishedPeriodUs = candidatePeriodUs;
    _sample.periodUs = candidatePeriodUs;
    _sample.timedOut = false;
    _sample.rpmInst =
        60000000.0f / (_config.pulsesPerRevolution * static_cast<float>(candidatePeriodUs));

    const bool countWindowReady = _sample.rpmCountWindow > 0.0f;
    const float comparisonDenominator = std::max(_sample.rpmCountWindow, 100.0f);
    const float mismatch =
        countWindowReady ? std::fabs(_sample.rpmInst - _sample.rpmCountWindow) /
                               comparisonDenominator
                         : 0.0f;

    const bool captureWindowHealthy =
        countWindowReady && (mismatch <= _config.qualityTolerance);

    if (countWindowReady) {
        if (captureWindowHealthy) {
            _captureGoodWindows = static_cast<uint8_t>(
                std::min<uint32_t>(_captureGoodWindows + 1U, 255U));
            _captureBadWindows = 0;
            if (_captureGoodWindows >= _config.captureHealthyRequireGoodWindows) {
                _captureHealthy = true;
            }
        } else {
            _captureBadWindows = static_cast<uint8_t>(
                std::min<uint32_t>(_captureBadWindows + 1U, 255U));
            _captureGoodWindows = 0;
            if (_captureBadWindows >= _config.captureHealthyRequireBadWindows) {
                _captureHealthy = false;
            }
        }
    }

    if (!countWindowReady) {
        _sample.rpmPub = _sample.rpmInst;
        _sample.rpmSource = 1;
        _sample.qualityOk = false;
    } else if (_captureHealthy && captureWindowHealthy) {
        _sample.rpmPub = _sample.rpmInst;
        _sample.rpmSource = 1;
        _sample.qualityOk = true;
    } else {
        // Prefer the count-window RPM whenever the instantaneous period branch
        // still disagrees materially with the robust frequency estimate.
        _sample.rpmPub = _sample.rpmCountWindow;
        _sample.rpmSource = 2;
        _sample.qualityOk = captureWindowHealthy;
    }

    _sample.pcntHealthy = _pcntHealthy;

    if (_sample.rpmEma <= 0.0f) {
        _sample.rpmEma = _sample.rpmPub;
    } else {
        // Smooth the published RPM so charts can show a stable trend while the
        // raw instantaneous and count-window estimates remain available.
        _sample.rpmEma += _config.emaAlpha * (_sample.rpmPub - _sample.rpmEma);
    }
}
