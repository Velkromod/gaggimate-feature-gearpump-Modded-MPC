// PressureController.h
#ifndef PRESSURE_CONTROLLER_H
#define PRESSURE_CONTROLLER_H

#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif

#include "SimpleKalmanFilter/SimpleKalmanFilter.h"
#include <algorithm>
#include <cmath>

class PressureController {
  private:
    // Generic first-order low-pass helper reused by estimator / telemetry / FF path.
    static void applyLowPassFilter(float *filteredValue, float rawValue, float cutoffFreq, float dt);

  public:
    enum class ControlMode { POWER, PRESSURE, FLOW };

    PressureController(float dt, float *_rawPressureSetpoint, float *_rawFlowSetpoint, float *sensorOutput,
                       float *controllerOutput, int *valveStatus);

    void initSetpointFilter(float val = 0.0f);

    void setFlowLimit(float lim) { /* Flow limit not currently implemented */ }
    void setPressureLimit(float lim) { /* Pressure limit not currently implemented */ }

    void update(ControlMode mode);
    void tare();
    void reset();

    float getCoffeeOutputEstimate() { return std::max(0.0f, _coffeeOutput); }
    void setPumpFlowCoeff(float oneBarFlow, float nineBarFlow);
    void setPumpFlowPolyCoeffs(float a, float b, float c, float d);
    float getPumpFlowRate() { return exportPumpFlowRate; }
    float getCoffeeFlowRate() { return *_valveStatus == 1 ? _coffeeFlowRate : 0.0f; }
    float getPuckResistance() { return _puckResistance; }

    // Debug / telemetry getters
    float getRawPressure() const { return _rawPressure ? *_rawPressure : 0.0f; }
    float getFilteredPressure() const { return _filteredPressureSensor; }

    float getRawPressureSetpoint() const { return _rawPressureSetpoint ? *_rawPressureSetpoint : 0.0f; }
    float getFilteredPressureSetpoint() const { return _filteredSetpoint; }
    float getFilteredSetpointDerivative() const { return _filteredSetpointDerivative; }

    float getControlOutput() const { return _ctrlOutput ? *_ctrlOutput : 0.0f; }
    float getFilteredPressureDerivative() const { return _filteredPressureDerivative; }
    float getPumpDutyCycleInternal() const { return _pumpDutyCycle; }

    // Existing limiter telemetry semantics:
    // raw = post-clamp, pre-slew
    float getRawPressureControlOutput() const { return _lastRawPressureOutput; }
    float getAppliedPressureControlOutput() const { return _lastAppliedPressureOutput; }
    float getPressureLimiterDelta() const { return _lastRawPressureOutput - _lastAppliedPressureOutput; }
    bool isPressureLimiterActiveUp() const { return _pressureLimiterActiveUp; }
    bool isPressureLimiterActiveDown() const { return _pressureLimiterActiveDown; }

    // Extended clamp telemetry
    float getUnclampedPressureControlOutput() const { return _lastUnclampedPressureOutput; }
    float getClampedPressureControlOutput() const { return _lastClampedPressureOutput; }
    float getPressureClampDelta() const { return _lastUnclampedPressureOutput - _lastClampedPressureOutput; }
    bool isPressureClampActiveHigh() const { return _pressureClampActiveHigh; }
    bool isPressureClampActiveLow() const { return _pressureClampActiveLow; }

    // Feedback / feedforward telemetry
    float getPressureFeedbackOutput() const { return _lastFeedbackOutput; }
    float getPressureFeedforwardHoldOutput() const { return _lastFeedforwardHoldOutput; }
    float getPressureFeedforwardDynamicOutput() const { return _lastFeedforwardDynamicOutput; }
    float getPressureFeedforwardDynamicRawRequest() const { return _lastFeedforwardDynamicRawRequest; }
    float getPressureFeedforwardPressureWeight() const { return _lastFeedforwardPressureWeight; }
    float getPressureFeedforwardAboveWeight() const { return _lastFeedforwardAboveWeight; }
    float getPressureFeedforwardGamma() const { return _lastFeedforwardGamma; }
    float getPressureFeedforwardTotalOutput() const { return _lastFeedforwardTotalOutput; }
    bool isRampHoldActive() const { return _rampHoldActive; }
    float getPressureDropRateActive() const { return _activePressureDropRate; }

    // Stage 0/1 shadow-MPC telemetry.
    // This branch never commands the actuator directly; it is only used to
    // evaluate model quality and suggested trim moves against the baseline.
    void setMpcShadowEnabled(bool enabled) {
        _mpcShadowEnabled = enabled;
        if (!enabled)
            resetShadowMpc();
    }
    bool isMpcShadowEnabled() const { return _mpcShadowEnabled; }
    float getRecipePressureSetpoint() const { return getRawPressureSetpoint(); }
    float getControlPressureSetpoint() const { return _filteredSetpoint; }
    float getMpcShadowSuggestedOutput() const { return _mpcShadowSuggestedOutput; }
    float getMpcShadowSteadyStateOutput() const { return _mpcShadowSteadyStateOutput; }
    float getMpcShadowTrimOutput() const { return _mpcShadowTrimOutput; }
    float getMpcShadowPredictedNextPressure() const { return _mpcShadowPredictedNextPressure; }
    float getMpcShadowPredictedTerminalPressure() const { return _mpcShadowPredictedTerminalPressure; }
    float getMpcShadowEstimatedOutflow() const { return _mpcShadowEstimatedOutflow; }
    float getMpcShadowEstimatedOutflowRaw() const { return _mpcShadowEstimatedOutflowRaw; }
    float getMpcShadowResidual() const { return _mpcShadowResidual; }
    float getMpcShadowResidualBias() const { return _mpcShadowResidualBias; }
    float getMpcShadowCost() const { return _mpcShadowCost; }

    float getErrorIntegral() const { return _errorIntegral; }

    void setDeadVolume(float deadVol) { _puckSaturatedVolume = deadVol; }

  private:
    float getPumpDutyCycleForPressure();
    void virtualScale();
    void filterSensor();
    void filterSetpoint(float rawSetpoint);

    float pumpFlowModel(float alpha = 100.0f) const;
    float getAvailableFlow() const;
    float getAvailableFlowAtPressure(float pressure) const;
    float getPumpDutyCycleForFlowRate() const;

    // Feedforward / weighting helpers
    float getScheduledSetpointWeight(float pressureRef) const;
    float getPressureFeedforwardHold(float pressureRef) const;
    float getPressureFeedforwardDynamicUnfiltered(float pressureRef, float pressureRefDerivative, float signedError);

    // Shadow-MPC helpers (Stage 0/1).
    void updateShadowMpc(float actualOutputPct);
    void resetShadowMpc();
    float predictPressureOneStep(float pressure, float dutyPct, float estimatedOutflow) const;

    float _dt = 1.0f; // Controller sampling period (seconds)

    // Input/output pointers
    float *_rawPressureSetpoint = nullptr; // Pressure profile current setpoint/limit (bar)
    float *_rawFlowSetpoint = nullptr;     // Flow profile current setpoint/limit (ml/s)
    float *_rawPressure = nullptr;         // Raw pressure measurement from sensor (bar)
    float *_ctrlOutput = nullptr;          // Controller output power ratio (0-100%)
    int *_valveStatus = nullptr;           // 3-way valve status (group head open/closed)

    // Filtered values
    float _filteredPressureSensor = 0.0f;     // Filtered pressure sensor reading (bar)
    float _filteredSetpoint = 0.0f;           // Filtered pressure setpoint (bar)
    float _filteredSetpointDerivative = 0.0f; // Derivative of filtered setpoint (bar/s)
    float _filteredPressureDerivative = 0.0f; // Derivative of filtered pressure (bar/s)

    // Setpoint filter parameters
    float _setpointFilterFreq = 1.6f;    // Setpoint filter cutoff frequency (Hz)
    float _setpointFilterDamping = 1.2f; // Setpoint filter damping ratio
    bool _setpointFilterInitialized = false;

    // === System parameters ===
    const float _systemCompliance = 1.4f;                            // System compliance (ml/bar)
    float _puckResistance = 1e7f;                                    // Initial estimate of puck resistance
    const float _maxPressure = 15.0f;                                // Maximum pressure (bar)
    const float _maxPressureRate = 13.0f;                            // Maximum pressure rate (bar/s)
    float _pumpFlowCoefficients[4] = {0.0f, 0.0f, -0.5854f, 10.79f}; // Pump flow polynomial coefficients

    // === Controller Gains ===
    float _commutationGain = 0.180f;    // Commutation gain
    float _convergenceGain = 1.03f;     // Convergence gain
    float _epsilonCoefficient = 0.40f;  // Limit band coefficient
    float _deadbandCoefficient = 0.06f; // Dead band coefficient
    float _integralGain = 0.08f;        // Integral gain (dt/tau)

    // ---------------------------------------------------------------------
    // 2-DOF-like setpoint weighting parameters.
    // Weighting remains intentionally disabled in this iteration.
    // ---------------------------------------------------------------------
    float _setpointWeightLowPressure = 1.0f;
    float _setpointWeightHighPressure = 1.0f;
    float _setpointWeightScheduleStartBar = 3.0f;
    float _setpointWeightScheduleEndBar = 9.0f;

    // ---------------------------------------------------------------------
    // Feedforward tuning parameters.
    // Static hold FF stays disabled.
    // Dynamic FF uses an above-target shutoff and a bounded gamma boost.
    // ---------------------------------------------------------------------
    float _feedforwardBiasPct = 0.0f;
    float _feedforwardPressureGainPctPerBar = 0.0f; // Hold FF disabled
    float _feedforwardHoldMaxPct = 0.0f;            // Hold FF disabled

    // Dynamic FF tuning - conservative recenter after the stronger balanced-light experiment.
    // Keep the FF-first architecture, but back off authority to recover a cleaner tradeoff.
    float _feedforwardRampGain = 0.34f;             // Conservative gain to avoid overdriving mid-rise
    float _feedforwardDynamicMaxPct = 8.2f;         // Slightly lower cap to keep mid-rise help while reducing roughness
    float _feedforwardDynamicFilterFreq = 1.2f;     // Slightly more filtering to reduce texture
    float _filteredFeedforwardDynamicOutput = 0.0f; // Filtered dynamic FF state
    float _lastFeedforwardDynamicAppliedOutput = 0.0f; // Slew-limited dynamic FF state

    // Dedicated slew limiter for the dynamic FF path itself
    float _feedforwardDynamicRiseRate = 70.0f;  // %/s
    float _feedforwardDynamicDropRate = 110.0f; // %/s

    // Smooth FF pressure window
    float _ffPressureGateStartBar = 1.3f;  // Start FF a bit earlier to support the lower mid-rise
    float _ffPressureGateFullBar = 3.7f;   // Reach full FF authority slightly earlier in the ramp
    float _ffPressureTaperStartBar = 7.0f; // Taper a touch earlier to calm the crest and late-rise texture
    float _ffPressureTaperEndBar = 9.0f;   // FF reaches zero near the peak

    // Above-target shutoff for FF.
    // FF stays active while pressure is below or near target, and fades out only when pressure rises above target.
    float _ffAboveGateStartBar = 0.05f; // bar above target where FF starts fading out
    float _ffAboveGateFullBar = 0.20f;  // shut FF off a bit sooner once pressure rises above target

    // Ramp-dependent gamma boost.
    // This scales FF during genuine rising references to offset premature feedback unloading.
    float _ffRampDerivGateStart = 0.20f; // bar/s where gamma starts ramping in
    float _ffRampDerivGateFull = 0.80f;  // bar/s where gamma reaches full effect
    float _ffGammaBoostMax = 0.14f;      // lower gamma to reduce rise roughness with puck-load variability

    // Optional downstream ramp-hold.
    // This does not add new authority; it only prevents the output from falling too fast
    // during a live upward pressure ramp while the system is still below target.
    bool _rampHoldEnabled = true;
    float _rampHoldDerivativeGateBarPerSec = 0.20f; // Activate only during real upward ramps
    float _rampHoldErrorGateBar = 0.05f;            // Require clear below-target condition
    float _rampHoldDropRate = 60.0f;                // %/s, slightly gentler hold than the prior experiment

    // === Controller states ===
    float _previousPressure = 0.0f; // Previous pressure reading (bar)
    float _errorIntegral = 0.0f;    // Integral of pressure error

    // Internal pressure-branch duty command normalized to 0..1 after clamp, before slew
    float _pumpDutyCycle = 0.0f;

    // Feedback / feedforward telemetry (all in %)
    float _lastFeedbackOutput = 0.0f;
    float _lastFeedforwardHoldOutput = 0.0f;
    float _lastFeedforwardDynamicOutput = 0.0f;
    float _lastFeedforwardDynamicRawRequest = 0.0f;
    float _lastFeedforwardPressureWeight = 0.0f;
    float _lastFeedforwardAboveWeight = 0.0f;
    float _lastFeedforwardGamma = 1.0f;
    float _lastFeedforwardTotalOutput = 0.0f;

    // Pre-clamp / clamp / limiter telemetry (all in %)
    float _lastUnclampedPressureOutput = 0.0f; // Before 0..100 clamp
    float _lastClampedPressureOutput = 0.0f;   // After 0..100 clamp, before slew

    // Backward-compatible meaning:
    // raw = clamped pre-slew, applied = post-slew
    float _lastRawPressureOutput = 0.0f;
    float _lastAppliedPressureOutput = 0.0f;

    bool _pressureClampActiveHigh = false;
    bool _pressureClampActiveLow = false;
    bool _pressureLimiterActiveUp = false;
    bool _pressureLimiterActiveDown = false;
    bool _rampHoldActive = false;
    float _activePressureDropRate = 0.0f;

    // Bidirectional output rate limiter
    const float _maxPressureOutputRiseRate = 190.0f; // percentage points per second
    const float _maxPressureOutputDropRate = 255.0f; // percentage points per second

    // ---------------------------------------------------------------------
    // Stage 0/1 shadow-MPC state.
    // This is deliberately lightweight and non-authoritative: it predicts
    // pressure with a simple local model and logs a suggested trim around the
    // baseline controller, but it never drives the pump directly.
    // ---------------------------------------------------------------------
    bool _mpcShadowEnabled = true;
    int _mpcShadowHorizonSteps = 6;
    float _mpcShadowSteadyStateBlend = 0.45f;
    float _mpcShadowTrimGainPctPerBar = 6.0f;
    float _mpcShadowTrimMaxPct = 5.0f;
    float _mpcShadowOutflowFilterFreq = 0.55f;
    float _mpcShadowResidualBiasFilterFreq = 0.18f;
    float _mpcShadowResidualBiasLimitBar = 0.06f;
    float _mpcShadowResidualBiasDeadbandBar = 0.003f;
    float _mpcShadowPressureDerivativeClipBarPerSec = 8.0f;
    float _mpcShadowPressureWeight = 1.0f;
    float _mpcShadowTerminalWeight = 2.0f;
    float _mpcShadowControlWeight = 0.002f;

    float _mpcShadowEstimatedOutflow = 0.0f;
    float _mpcShadowEstimatedOutflowRaw = 0.0f;
    float _mpcShadowSuggestedOutput = 0.0f;
    float _mpcShadowSteadyStateOutput = 0.0f;
    float _mpcShadowTrimOutput = 0.0f;
    float _mpcShadowPredictedNextPressure = 0.0f;
    float _mpcShadowPredictedTerminalPressure = 0.0f;
    float _mpcShadowResidual = 0.0f;
    float _mpcShadowResidualBias = 0.0f;
    float _mpcShadowCost = 0.0f;
    float _mpcShadowPreviousPrediction = 0.0f;
    bool _mpcShadowPreviousPredictionValid = false;

    // === Flow estimation ===
    float _waterThroughPuckFlowRate = 0.0f; // Water through puck flow rate (ml/s)
    float _pumpFlowRate = 0.0f;             // Pump flow rate (ml/s)
    float _pumpVolume = 0.0f;               // Total pump volume (ml)
    float _coffeeOutput = 0.0f;             // Total coffee output (ml)
    float _coffeeFlowRate = 0.0f;           // Coffee output flow rate (mL/s)
    float _lastFilteredPressure = 0.0f;     // Previous filtered pressure for derivative calculation
    float _filterEstimatorFrequency = 1.0f; // Filter frequency for estimator
    float _pressureFilterEstimator = 0.0f;
    float _puckSaturationVolume = 0.0f; // Total volume to saturate the puck (ml)
    float _puckSaturatedVolume = 45.0f; // Volume at puck saturation (ml)
    float _lastPuckConductance = 0.0f;  // Previous puck conductance for derivative calculation
    float _puckConductance = 0.0f;
    float _puckConductanceDerivative = 0.0f; // Derivative of puck conductance
    bool _puckState[3] = {};
    int _puckCounter = 0;

    float exportPumpFlowRate = 0.0f; // Exported value separated from internal state for cosmetic filtering
    SimpleKalmanFilter *_pressureKalmanFilter = nullptr;
};

#endif // PRESSURE_CONTROLLER_H
