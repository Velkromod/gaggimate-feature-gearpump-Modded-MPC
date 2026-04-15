#include "DimmedPump.h"

#include <ExtensionIOXL9555.hpp>
#include <GaggiMateController.h>

#include <algorithm>
#include <cmath>

#define MCP_VOLTAGE 5.0f

static ExtensionIOXL9555 extension;

char swTxBuffer[128];
char swRxBuffer[128];

// Effective output-resolution improvement for the integer-only _psm.set():
// first-order error-diffusion / sigma-delta temporal dithering.
// This keeps the instantaneous command integer, but preserves fractional power on average.
namespace {
float g_powerQuantizationResidual = 0.0f;
float g_lastPowerCommandPct = 0.0f;
int g_lastQuantizedPowerPct = 0;

int quantizePumpPowerForPSM(float desiredPowerPct) {
    float desired = std::clamp(desiredPowerPct, 0.0f, 100.0f);
    g_lastPowerCommandPct = desired;

    float shaped = desired + g_powerQuantizationResidual;
    int quantized = static_cast<int>(std::lround(shaped));
    quantized = std::clamp(quantized, 0, 100);

    g_powerQuantizationResidual = shaped - static_cast<float>(quantized);

    // Avoid carrying useless residual at hard rails.
    if ((quantized == 0 && desired <= 0.0f) || (quantized == 100 && desired >= 100.0f)) {
        g_powerQuantizationResidual = 0.0f;
    }

    g_lastQuantizedPowerPct = quantized;
    return quantized;
}

void printShotTelemetryLegend() {
    printf("# SHOT START\n");
    printf("# legend_version=1\n");
    printf("# time_base=ms_since_boot\n");
    printf("# units: pressure=bar, flow=ml_s, output=percent, derivative=bar_s\n");
    printf("# raw_p = raw pressure sensor reading\n");
    printf("# flt_p = filtered pressure used by controller\n");
    printf("# sp_raw = raw incoming pressure setpoint currently held by the profile engine\n");
    printf("# sp_flt = filtered pressure setpoint actually used internally by the controller\n");
    printf("# sp_recipe = visual recipe reference; ideal profile target for display/comparison\n");
    printf("# sp_ctrl = control reference after internal conditioning; command seen by control logic\n");
    printf("# sp_flt_d = derivative of filtered pressure setpoint\n");
    printf("# ctrl_out = final controller output command in percent\n");
    printf("# pump_duty = internal normalized pressure-branch duty estimate\n");
    printf("# pump_flow = estimated pump flow\n");
    printf("# coffee_flow = estimated outflow through puck / coffee flow\n");
    printf("# u_raw = pressure branch output after clamp and before final slew limiter\n");
    printf("# u_applied = pressure branch output after final slew limiter\n");
    printf("# u_delta = u_raw - u_applied; limiter intervention magnitude\n");
    printf("# limiter_active_up = 1 when rise slew limiter is active\n");
    printf("# limiter_active_down = 1 when drop slew limiter is active\n");
    printf("# error_integral = integral state of legacy pressure controller\n");
    printf("# u_unclamped_raw = controller output before 0..100 clamp\n");
    printf("# u_clamped_raw = controller output after 0..100 clamp and before slew limiter\n");
    printf("# u_clamp_delta = u_unclamped_raw - u_clamped_raw\n");
    printf("# clamp_active_high = 1 when output was clipped by upper clamp\n");
    printf("# clamp_active_low = 1 when output was clipped by lower clamp\n");
    printf("# u_fb = legacy pressure feedback contribution\n");
    printf("# u_ff_hold = static feedforward contribution\n");
    printf("# u_ff_dyn = dynamic feedforward contribution\n");
    printf("# u_ff_dyn_raw = raw dynamic feedforward request before FF-path LPF and slew limiting\n");
    printf("# ff_pressure_w = pressure-window weight applied to dynamic FF\n");
    printf("# ff_above_w = above-target suppression weight applied to dynamic FF\n");
    printf("# ff_gamma = ramp-dependent gamma multiplier applied to dynamic FF\n");
    printf("# u_ff_total = total feedforward contribution\n");
    printf("# ramp_hold_active = 1 when the downstream drop-rate hold is active during ramp-up\n");
    printf("# drop_rate_active = active downstream output drop-rate cap in percent per second\n");
    printf("# mpc_shadow_enabled = 1 when shadow MPC is being computed in parallel\n");
    printf("# mpc_u_shadow = total output suggested by shadow MPC; not applied to actuator in shadow stage\n");
    printf("# mpc_u_ss = steady-state component suggested by shadow MPC\n");
    printf("# mpc_u_trim = corrective trim component suggested by shadow MPC\n");
    printf("# mpc_p1_pred = one-step pressure prediction from shadow MPC model\n");
    printf("# mpc_pn_pred = terminal pressure prediction at end of shadow MPC horizon\n");
    printf("# mpc_qout_est = estimated outflow / load used by shadow MPC model\n");
    printf("# mpc_residual = measured minus predicted pressure residual for shadow MPC\n");
    printf("# mpc_cost = shadow MPC objective value\n");
    printf("# power_cmd = floating-point power command sent to pump stage before integer quantization\n");
    printf("# power_psm_quantized = integer power actually sent to PSM\n");
    printf("# power_quant_residual = sigma-delta residual kept for next quantization step\n");
    printf("ms,raw_p,flt_p,sp_raw,sp_flt,sp_recipe,sp_ctrl,sp_flt_d,ctrl_out,pump_duty,pump_flow,coffee_flow,"
           "u_raw,u_applied,u_delta,limiter_active_up,limiter_active_down,error_integral,"
           "u_unclamped_raw,u_clamped_raw,u_clamp_delta,clamp_active_high,clamp_active_low,"
           "u_fb,u_ff_hold,u_ff_dyn,u_ff_dyn_raw,ff_pressure_w,ff_above_w,ff_gamma,u_ff_total,ramp_hold_active,drop_rate_active,"
           "mpc_shadow_enabled,mpc_u_shadow,mpc_u_ss,mpc_u_trim,mpc_p1_pred,mpc_pn_pred,mpc_qout_est,mpc_residual,mpc_cost,"
           "power_cmd,power_psm_quantized,power_quant_residual\n");
}
} // namespace

uint8_t read_scl(const SoftWire *i2c) {
    uint8_t value = extension.digitalRead(ExtensionIOXL9555::IO0);
    ESP_LOGV("MCP4725", "Read SCL: %d", value);
    return value;
}

uint8_t read_sda(const SoftWire *i2c) {
    uint8_t value = extension.digitalRead(ExtensionIOXL9555::IO1);
    ESP_LOGV("MCP4725", "Read SDA: %d", value);
    return value;
}

void scl_high(const SoftWire *i2c) {
    extension.pinMode(ExtensionIOXL9555::IO0, INPUT);
    ESP_LOGV("MCP4725", "Release SCL");
}

void sda_high(const SoftWire *i2c) {
    extension.pinMode(ExtensionIOXL9555::IO1, INPUT);
    ESP_LOGV("MCP4725", "Release SDA");
}

void scl_low(const SoftWire *i2c) {
    extension.pinMode(ExtensionIOXL9555::IO0, OUTPUT);
    extension.digitalWrite(ExtensionIOXL9555::IO0, LOW);
    ESP_LOGV("MCP4725", "Write SCL: %d", 0);
}

void sda_low(const SoftWire *i2c) {
    extension.pinMode(ExtensionIOXL9555::IO1, OUTPUT);
    extension.digitalWrite(ExtensionIOXL9555::IO1, LOW);
    ESP_LOGV("MCP4725", "Write SDA: %d", 0);
}

DimmedPump::DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressure_sensor, uint8_t scl_pin, uint8_t sda_pin)
    : _ssr_pin(ssr_pin),
      _sense_pin(sense_pin),
      _psm(_sense_pin, _ssr_pin, 100, FALLING, 2, 4),
      _pressureSensor(pressure_sensor),
      _pressureController(0.03f, &_ctrlPressure, &_ctrlFlow, &_currentPressure, &_controllerPower, &_valveStatus) {
    _psm.set(0);

    if (!extension.init(Wire, sda_pin, scl_pin, XL9555_UNKOWN_ADDRESS)) {
        ESP_LOGE(LOG_TAG, "Failed to initialize extension I2C bus");
    } else {
        ESP_LOGI(LOG_TAG, "Initialized extension");
        extension.setClock(1000000L);

        i2c = new SoftWire(0, 0);
        i2c->setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
        i2c->setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
        i2c->setReadScl(read_scl);
        i2c->setReadSda(read_sda);
        i2c->setSetSclHigh(scl_high);
        i2c->setSetSdaHigh(sda_high);
        i2c->setSetSclLow(scl_low);
        i2c->setSetSdaLow(sda_low);
        i2c->setTimeout_ms(200);
        i2c->setDelay_us(20);
        i2c->begin();

        delay(500);

        mcp = new MCP4725(0x60, i2c);
        if (!mcp->begin()) {
            ESP_LOGE(LOG_TAG, "Failed to initialize MCP4725");
        } else {
            ESP_LOGI(LOG_TAG, "MCP4725 initialized");
            mcp->setMaxVoltage(MCP_VOLTAGE);
            mcp->setPercentage(0);
        }
    }
}

void DimmedPump::setup() {
    _cps = _psm.cps();
    if (_cps > 70) {
        _cps = _cps / 2;
    }

    ESP_LOGI(LOG_TAG, "CSV debug logging enabled");

    xTaskCreate(loopTask, "DimmedPump::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void DimmedPump::loop() {
    _currentPressure = _pressureSensor->getRawPressure();
    updatePower();
    _currentFlow = _pressureController.getPumpFlowRate();

    bool shotActive =
        _debugLogEnabled &&
        (_mode != ControlMode::POWER) &&
        (_ctrlPressure > 0.2f || _ctrlFlow > 0.05f);

    if (shotActive && !_telemetryShotActive) {
        _telemetryShotActive = true;
        _lastDebugLogMs = 0;

        printf("=== SHOT TELEMETRY START ===\n");
        printShotTelemetryLegend();
        fflush(stdout);
    }

    if (!shotActive && _telemetryShotActive) {
        printf("=== SHOT TELEMETRY END ===\n");
        fflush(stdout);
        _telemetryShotActive = false;
    }

    if (shotActive) {
        uint32_t now = millis();
        if (now - _lastDebugLogMs >= 100) {
            _lastDebugLogMs = now;

            printf(
                "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.3f,%.3f,"
                "%.2f,%.2f,%.2f,%d,%d,%.5f,"
                "%.2f,%.2f,%.2f,%d,%d,"
                "%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.2f,%d,%.2f,"
                "%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                "%.2f,%d,%.4f\n",
                now,
                _pressureController.getRawPressure(),
                _pressureController.getFilteredPressure(),
                _pressureController.getRawPressureSetpoint(),
                _pressureController.getFilteredPressureSetpoint(),
                _pressureController.getRecipePressureSetpoint(),
                _pressureController.getControlPressureSetpoint(),
                _pressureController.getFilteredSetpointDerivative(),
                _pressureController.getControlOutput(),
                _pressureController.getPumpDutyCycleInternal(),
                _pressureController.getPumpFlowRate(),
                _pressureController.getCoffeeFlowRate(),

                _pressureController.getRawPressureControlOutput(),
                _pressureController.getAppliedPressureControlOutput(),
                _pressureController.getPressureLimiterDelta(),
                _pressureController.isPressureLimiterActiveUp() ? 1 : 0,
                _pressureController.isPressureLimiterActiveDown() ? 1 : 0,
                _pressureController.getErrorIntegral(),

                _pressureController.getUnclampedPressureControlOutput(),
                _pressureController.getClampedPressureControlOutput(),
                _pressureController.getPressureClampDelta(),
                _pressureController.isPressureClampActiveHigh() ? 1 : 0,
                _pressureController.isPressureClampActiveLow() ? 1 : 0,

                _pressureController.getPressureFeedbackOutput(),
                _pressureController.getPressureFeedforwardHoldOutput(),
                _pressureController.getPressureFeedforwardDynamicOutput(),
                _pressureController.getPressureFeedforwardDynamicRawRequest(),
                _pressureController.getPressureFeedforwardPressureWeight(),
                _pressureController.getPressureFeedforwardAboveWeight(),
                _pressureController.getPressureFeedforwardGamma(),
                _pressureController.getPressureFeedforwardTotalOutput(),
                _pressureController.isRampHoldActive() ? 1 : 0,
                _pressureController.getPressureDropRateActive(),

                _pressureController.isMpcShadowEnabled() ? 1 : 0,
                _pressureController.getMpcShadowSuggestedOutput(),
                _pressureController.getMpcShadowSteadyStateOutput(),
                _pressureController.getMpcShadowTrimOutput(),
                _pressureController.getMpcShadowPredictedNextPressure(),
                _pressureController.getMpcShadowPredictedTerminalPressure(),
                _pressureController.getMpcShadowEstimatedOutflow(),
                _pressureController.getMpcShadowResidual(),
                _pressureController.getMpcShadowCost(),

                g_lastPowerCommandPct,
                g_lastQuantizedPowerPct,
                g_powerQuantizationResidual
            );
            fflush(stdout);
        }
    }
}

void DimmedPump::setPower(float setpoint) {
    ESP_LOGV(LOG_TAG, "Setting power to %2f", setpoint);
    _ctrlPressure = setpoint > 0 ? 20.0f : 0.0f;
    _mode = ControlMode::POWER;
    _power = std::clamp(setpoint, 0.0f, 100.0f);
    _controllerPower = _power; // Feed manual control back into pressure controller

    if (_power == 0.0f) {
        _currentFlow = 0.0f;
    }

    int quantizedPower = quantizePumpPowerForPSM(_power);
    _psm.set(quantizedPower);

    // Keep full float resolution on DAC side.
    mcp->setVoltage(MCP_VOLTAGE * _power / 100.0f);
}

float DimmedPump::getCoffeeVolume() { return _pressureController.getCoffeeOutputEstimate(); }

float DimmedPump::getPumpFlow() { return _currentFlow; }

float DimmedPump::getPuckFlow() { return _pressureController.getCoffeeFlowRate(); }

float DimmedPump::getPuckResistance() { return _pressureController.getPuckResistance(); }

void DimmedPump::tare() {
    _pressureController.tare();
    _pressureController.reset();
    g_powerQuantizationResidual = 0.0f;
}

void DimmedPump::loopTask(void *arg) {
    auto *pump = static_cast<DimmedPump *>(arg);
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        pump->loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(30));
    }
}

void DimmedPump::updatePower() {
    _pressureController.update(static_cast<PressureController::ControlMode>(_mode));

    if (_mode != ControlMode::POWER) {
        _power = _controllerPower;
    }

    int quantizedPower = quantizePumpPowerForPSM(_power);
    _psm.set(quantizedPower);

    // Keep DAC at full float resolution.
    mcp->setVoltage(MCP_VOLTAGE * _power / 100.0f);
}

void DimmedPump::setFlowTarget(float targetFlow, float pressureLimit) {
    _mode = ControlMode::FLOW;
    _ctrlFlow = targetFlow;
    _ctrlPressure = pressureLimit;
    _pressureController.setPressureLimit(pressureLimit);
}

void DimmedPump::setPressureTarget(float targetPressure, float flowLimit) {
    _mode = ControlMode::PRESSURE;
    _ctrlFlow = flowLimit;
    _ctrlPressure = targetPressure;
    _pressureController.setFlowLimit(flowLimit);
}

void DimmedPump::setValveState(bool open) { _valveStatus = open; }

void DimmedPump::setPumpFlowCoeff(float oneBarFlow, float nineBarFlow) {
    _pressureController.setPumpFlowCoeff(oneBarFlow, nineBarFlow);
}

void DimmedPump::setPumpFlowPolyCoeffs(float a, float b, float c, float d) {
    _pressureController.setPumpFlowPolyCoeffs(a, b, c, d);
}
