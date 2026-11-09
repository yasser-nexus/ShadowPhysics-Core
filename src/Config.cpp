// =============================================================================
//  FH4RPO - Config.cpp
//
//  A dependency free INI reader/writer.  The parser is deliberately defensive:
//  a malformed line, an unknown key or a wildly out of range value can only
//  ever fall back to the default - it can never abort a load or leave a
//  tunable in a state the solver cannot handle.
// =============================================================================
#include "fh4rpo/Config.hpp"
#include "fh4rpo/Core.hpp"
#include "fh4rpo/Log.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <sys/stat.h>

namespace fh4rpo {

namespace {

// -----------------------------------------------------------------------------
//  Minimal INI document: section -> (key -> value)
// -----------------------------------------------------------------------------
using KeyMap = std::unordered_map<std::string, std::string>;
using SectionMap = std::unordered_map<std::string, KeyMap>;

std::string Trim(const std::string& s)
{
    std::size_t start = 0;
    while (start < s.size() && (s[start] == ' ' || s[start] == '\t' || s[start] == '\r' || s[start] == '\n'))
        ++start;
    std::size_t end = s.size();
    while (end > start && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r' || s[end - 1] == '\n'))
        --end;
    return s.substr(start, end - start);
}

std::string LowerCopy(const std::string& s)
{
    std::string out = s;
    for (char& c : out)
        c = static_cast<char>(::tolower(static_cast<unsigned char>(c)));
    return out;
}

bool ParseIni(const std::string& text, SectionMap& out)
{
    SectionMap sections;
    std::string current;
    std::size_t pos = 0;

    while (pos <= text.size())
    {
        std::size_t eol = text.find('\n', pos);
        if (eol == std::string::npos)
            eol = text.size();
        std::string line = Trim(text.substr(pos, eol - pos));
        pos = eol + 1;

        if (line.empty())
            continue;
        if (line[0] == ';' || line[0] == '#')
            continue;

        if (line.front() == '[' && line.back() == ']')
        {
            current = LowerCopy(Trim(line.substr(1, line.size() - 2)));
            sections[current];      // create even if empty
            continue;
        }

        const std::size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;

        const std::string key = LowerCopy(Trim(line.substr(0, eq)));
        std::string value = Trim(line.substr(eq + 1));

        // strip an inline comment that starts after whitespace
        const std::size_t comment = value.find_first_of(";#");
        if (comment != std::string::npos)
            value = Trim(value.substr(0, comment));

        if (!key.empty())
            sections[current][key] = value;
    }

    out = std::move(sections);
    return true;
}

// -----------------------------------------------------------------------------
//  Typed getters - always return a sanitised value, never throw.
// -----------------------------------------------------------------------------
class Reader
{
public:
    Reader(const SectionMap& map, const char* section) : section_(section)
    {
        const auto it = map.find(LowerCopy(section));
        if (it != map.end())
            keys_ = &it->second;
    }

    bool GetBool(const char* key, bool fallback) const
    {
        const std::string* v = Find(key);
        if (!v)
            return fallback;
        const std::string low = LowerCopy(*v);
        if (low == "1" || low == "true" || low == "yes" || low == "on")
            return true;
        if (low == "0" || low == "false" || low == "no" || low == "off")
            return false;
        return fallback;
    }

    float GetFloat(const char* key, float fallback) const
    {
        const std::string* v = Find(key);
        if (!v)
            return fallback;
        char* end = nullptr;
        const double parsed = std::strtod(v->c_str(), &end);
        if (end == v->c_str() || !IsFinite(parsed))
            return fallback;
        return static_cast<float>(parsed);
    }

    int GetInt(const char* key, int fallback) const
    {
        const std::string* v = Find(key);
        if (!v)
            return fallback;
        char* end = nullptr;
        const long parsed = std::strtol(v->c_str(), &end, 10);
        if (end == v->c_str())
            return fallback;
        return static_cast<int>(parsed);
    }

    std::string GetString(const char* key, const std::string& fallback) const
    {
        const std::string* v = Find(key);
        return v ? *v : fallback;
    }

    // Comma separated list of floats, e.g. "1.0, 1.02, 0.99"
    int GetFloatList(const char* key, float* out, int maxCount, int expectedCount) const
    {
        const std::string* v = Find(key);
        if (!v || maxCount <= 0)
            return 0;

        int written = 0;
        const char* cursor = v->c_str();
        while (written < maxCount && cursor && *cursor)
        {
            char* end = nullptr;
            const double parsed = std::strtod(cursor, &end);
            if (end == cursor)
                break;
            out[written++] = IsFinite(parsed) ? static_cast<float>(parsed) : 0.0f;
            cursor = end;
            while (*cursor == ',' || *cursor == ' ' || *cursor == '\t')
                ++cursor;
        }
        return (expectedCount > 0 && written != expectedCount) ? written : written;
    }

    // "1.0,1.02,0.99; 1.0,1.01,0.98" -> rows*cols matrix
    bool GetMatrix(const char* key, float* out, int rows, int cols) const
    {
        const std::string* v = Find(key);
        if (!v)
            return false;

        std::vector<float> flat;
        const char* cursor = v->c_str();
        while (cursor && *cursor)
        {
            char* end = nullptr;
            const double parsed = std::strtod(cursor, &end);
            if (end == cursor)
                break;
            flat.push_back(IsFinite(parsed) ? static_cast<float>(parsed) : 1.0f);
            cursor = end;
            while (*cursor == ',' || *cursor == ' ' || *cursor == '\t')
                ++cursor;
        }

        if (static_cast<int>(flat.size()) != rows * cols)
            return false;
        for (int i = 0; i < rows * cols; ++i)
            out[i] = flat[static_cast<std::size_t>(i)];
        return true;
    }

private:
    const std::string* Find(const char* key) const
    {
        if (keys_ == nullptr)
            return nullptr;
        const auto it = keys_->find(LowerCopy(key));
        return (it == keys_->end()) ? nullptr : &it->second;
    }

    const char* section_;
    const KeyMap* keys_ = nullptr;
};

std::string HexOrZero(unsigned long long value)
{
    if (value == 0)
        return "0";
    char buffer[32] = {};
    std::snprintf(buffer, sizeof(buffer), "0x%llX", value);
    return buffer;
}

// Accepts "0x7FF6..." and plain decimal, returns 0 on anything unparsable.
unsigned long long ParseAddress(const std::string& text)
{
    if (text.empty())
        return 0;
    char* end = nullptr;
    const unsigned long long value = std::strtoull(text.c_str(), &end, 0);
    if (end == text.c_str())
        return 0;
    return value;
}

std::string FormatFloat(float value)
{
    char buffer[64] = {};
    std::snprintf(buffer, sizeof(buffer), "%.6g", static_cast<double>(value));
    return buffer;
}

std::string FormatFloatList(const float* values, int count)
{
    std::string out;
    for (int i = 0; i < count; ++i)
    {
        if (i)
            out += ", ";
        out += FormatFloat(values[i]);
    }
    return out;
}

std::string FormatMatrix(const float* values, int rows, int cols)
{
    std::string out;
    for (int r = 0; r < rows; ++r)
    {
        if (r)
            out += "; ";
        for (int c = 0; c < cols; ++c)
        {
            if (c)
                out += ", ";
            out += FormatFloat(values[r * cols + c]);
        }
    }
    return out;
}

// Writes the complete default document - also used as the in-game "reset".
std::string BuildDefaultDocument(const Config& c)
{
    std::string d;
    auto section = [&d](const char* name) { d += "\n["; d += name; d += "]\n"; };
    auto kv = [&d](const char* key, const std::string& value)
    {
        d += key; d += "="; d += value; d += "\n";
    };
    auto kf = [&](const char* key, float value) { kv(key, FormatFloat(value)); };
    auto kb = [&](const char* key, bool value) { kv(key, value ? "true" : "false"); };
    auto ki = [&](const char* key, int value) { kv(key, std::to_string(value)); };

    d += "; ============================================================================\n";
    d += "; FH4 Realistic Physics Overhaul - configuration\n";
    d += "; Hot reload: edit this file and it is picked up within one second.\n";
    d += "; In game: F2 master on/off, F3 passive mode, F4 panic unload, F5 reload\n";
    d += "; F8 writes a state dump.\n";
    d += "; ============================================================================\n";

    section("General");
    kv("module_name", c.moduleName);
    ki("log_level", c.debug.logLevel);
    ki("overlay_mode", c.debug.overlayMode);
    kf("overlay_interval_s", c.debug.overlayIntervalS);

    section("Vehicle");
    kf("mass_kg", c.vehicle.massKg);
    kf("wheelbase_m", c.vehicle.wheelbaseM);
    kf("front_track_m", c.vehicle.frontTrackM);
    kf("rear_track_m", c.vehicle.rearTrackM);
    kf("cg_height_m", c.vehicle.cgHeightM);
    kf("weight_distribution", c.vehicle.weightDistribution);
    kf("wheel_radius_front_m", c.vehicle.wheelRadiusFrontM);
    kf("wheel_radius_rear_m", c.vehicle.wheelRadiusRearM);
    kf("roll_inertia_kgm2", c.vehicle.rollInertiaKgM2);
    kf("pitch_inertia_kgm2", c.vehicle.pitchInertiaKgM2);
    kf("yaw_inertia_kgm2", c.vehicle.yawInertiaKgM2);
    kf("roll_stiffness_nm_per_rad", c.vehicle.rollStiffnessNmPerRad);
    kf("pitch_stiffness_nm_per_rad", c.vehicle.pitchStiffnessNmPerRad);
    kf("roll_damping_nms_per_rad", c.vehicle.rollDampingNmsPerRad);
    kf("pitch_damping_nms_per_rad", c.vehicle.pitchDampingNmsPerRad);
    kf("anti_roll_front_nm_per_rad", c.vehicle.antiRollBarFrontNmPerRad);
    kf("anti_roll_rear_nm_per_rad", c.vehicle.antiRollBarRearNmPerRad);
    kf("max_suspension_travel_m", c.vehicle.maxSuspensionTravelM);
    kf("ride_height_m", c.vehicle.rideHeightM);
    kf("front_tire_width_m", c.vehicle.frontTireWidthM);
    kf("rear_tire_width_m", c.vehicle.rearTireWidthM);
    ki("drive_layout", c.vehicle.driveLayout);

    section("Tires");
    kf("mu_front", c.tire.muNominalFront);
    kf("mu_rear", c.tire.muNominalRear);
    kf("b_long", c.tire.bLong);
    kf("c_long", c.tire.cLong);
    kf("d_long_scale", c.tire.dLongScale);
    kf("e_long", c.tire.eLong);
    kf("b_lat", c.tire.bLat);
    kf("c_lat", c.tire.cLat);
    kf("e_lat", c.tire.eLat);
    kf("load_sensitivity", c.tire.loadSensitivity);
    kf("load_reference_n", c.tire.loadReferenceN);
    kf("camber_gain_per_deg", c.tire.camberGainPerDeg);
    kf("slip_angle_peak_deg", c.tire.slipAnglePeakDeg);
    kf("slip_ratio_peak", c.tire.slipRatioPeak);
    kf("relaxation_length_m", c.tire.relaxationLengthM);
    kf("temperature_optimum_c", c.tire.temperatureOptimumC);
    kf("temperature_window_c", c.tire.temperatureWindowC);
    kf("temperature_grip_floor", c.tire.temperatureGripFloor);
    kf("wear_grip_floor", c.tire.wearGripFloor);
    kf("wear_rate", c.tire.wearRate);
    kf("heat_generation", c.tire.heatGeneration);
    kf("cooling_rate", c.tire.coolingRate);
    kf("surface_friction_wet", c.tire.surfaceFrictionWet);
    kf("surface_friction_grass", c.tire.surfaceFrictionGrass);
    kf("surface_friction_dirt", c.tire.surfaceFrictionDirt);
    kf("surface_friction_snow", c.tire.surfaceFrictionSnow);

    section("FrictionMatrix");
    kb("enabled", c.friction.enabled);
    kb("asymmetric", c.friction.asymmetric);
    kb("per_axle", c.friction.perAxle);
    kb("use_load_axis", c.friction.useLoadAxis);
    kf("snap_deg", c.friction.snapDeg);
    kf("fade_rate", c.friction.fadeRate);
    kv("front_angle_keys_deg", FormatFloatList(c.friction.frontAngleKeysDeg, 6));
    kv("rear_angle_keys_deg", FormatFloatList(c.friction.rearAngleKeysDeg, 6));
    kv("load_keys_n", FormatFloatList(c.friction.loadKeysN, 4));
    kv("front_values", FormatMatrix(&c.friction.frontValues[0][0], 6, 4));
    kv("rear_values", FormatMatrix(&c.friction.rearValues[0][0], 6, 4));

    section("WeightTransfer");
    kb("enabled", c.weight.enabled);
    kf("long_transfer_gain", c.weight.longTransferGain);
    kf("lat_transfer_gain", c.weight.latTransferGain);
    kf("pitch_accel_gain", c.weight.pitchAccelGain);
    kf("roll_accel_gain", c.weight.rollAccelGain);
    kf("geometry_gain", c.weight.geometryGain);
    kf("jacking_gain", c.weight.jackingGain);
    kf("min_corner_load_n", c.weight.minCornerLoadN);
    kf("load_rate_limit_n", c.weight.loadRateLimitN);
    kf("anti_dive_gain", c.weight.antiDiveGain);
    kf("anti_squat_gain", c.weight.antiSquatGain);

    section("Aero");
    kb("enabled", c.aero.enabled);
    kb("override_game_downforce", c.aero.overrideGameDownforce);
    kf("air_density_kgm3", c.aero.airDensityKgM3);
    kf("lift_coefficient_front", c.aero.liftCoefficientFront);
    kf("lift_coefficient_rear", c.aero.liftCoefficientRear);
    kf("frontal_area_m2", c.aero.frontalAreaM2);
    kf("drag_coefficient", c.aero.dragCoefficient);
    kf("ground_effect_height_m", c.aero.groundEffectHeightM);
    kf("ground_effect_gain", c.aero.groundEffectGain);
    kf("stall_speed_mps", c.aero.stallSpeedMps);
    kf("stall_falloff", c.aero.stallFalloff);
    kf("balance_split", c.aero.balanceSplit);
    kf("yaw_sensitivity", c.aero.yawSensitivity);
    kf("smooth_time_s", c.aero.smoothTimeS);
    kf("max_downforce_n", c.aero.maxDownforceN);
    kf("max_drag_n", c.aero.maxDragN);

    section("Drivetrain");
    kb("enabled", c.drivetrain.enabled);
    kf("torque_split_front", c.drivetrain.torqueSplitFront);
    kf("engine_braking_gain", c.drivetrain.engineBrakingGain);
    kf("clutch_engagement", c.drivetrain.clutchEngagement);
    kf("driveline_inertia_kgm2", c.drivetrain.drivelineInertiaKgM2);
    kf("rear_bias_on_launch", c.drivetrain.rearBiasOnLaunch);
    kf("launch_speed_mps", c.drivetrain.launchSpeedMps);
    kf("max_engine_torque_nm", c.drivetrain.maxEngineTorqueNm);
    kf("torque_curve_peak_rpm", c.drivetrain.torqueCurvePeakRpm);
    kf("torque_curve_width_rpm", c.drivetrain.torqueCurveWidthRpm);

    section("Brakes");
    kb("enabled", c.brakes.enabled);
    kf("max_brake_torque_nm", c.brakes.maxBrakeTorqueNm);
    kf("front_bias", c.brakes.frontBias);
    kf("abs_slip_threshold", c.brakes.absSlipThreshold);
    kf("abs_cycle_hz", c.brakes.absCycleHz);
    kf("brake_fade_start_c", c.brakes.brakeFadeStartC);
    kf("brake_fade_rate", c.brakes.brakeFadeRate);
    kf("handbrake_torque_nm", c.brakes.handbrakeTorqueNm);

    section("Steering");
    kb("enabled", c.steering.enabled);
    kf("max_steer_angle_deg", c.steering.maxSteerAngleDeg);
    kf("speed_sensitivity", c.steering.speedSensitivity);
    kf("min_angle_at_speed_deg", c.steering.minAngleAtSpeedDeg);
    kf("ackermann_gain", c.steering.ackermannGain);
    kf("counter_steer_assist", c.steering.counterSteerAssist);
    kf("assist_slip_deadzone_deg", c.steering.assistSlipDeadzoneDeg);
    kf("autosteer_rate", c.steering.autosteerRate);
    kf("return_rate", c.steering.returnRate);
    kf("ffb_gain", c.steering.ffbGain);

    section("Memory");
    kb("enable_write_guards", c.memory.enableWriteGuards);
    kb("enable_self_test", c.memory.enableSelfTest);
    kb("verify_hook_targets", c.memory.verifyHookTargets);
    kf("resolve_interval_s", c.memory.resolveIntervalS);
    ki("scanner_alignment", c.memory.scannerAlignment);
    kb("enable_debug_privilege", c.memory.enableDebugPrivilege);
    kb("guard_on_suspend", c.memory.guardOnSuspend);

    section("Telemetry");
    kb("enabled", c.telemetry.enabled);
    ki("port", c.telemetry.port);
    ki("bind_port_fallback", c.telemetry.bindPortFallback);
    kb("bind_loopback_only", c.telemetry.bindLoopbackOnly);
    kf("timeout_seconds", c.telemetry.timeoutSeconds);
    kf("speed_filter_hz", c.telemetry.speedFilterHz);
    kb("auto_derive_spec", c.telemetry.autoDeriveSpec);
    kb("broadcast_assist", c.telemetry.broadcastAssist);
    ki("broadcast_port", c.telemetry.broadcastPort);

    section("Physics");
    kb("enabled", c.physics.enabled);
    kb("passive_mode_allowed", c.physics.passiveModeAllowed);
    kf("fixed_step_hz", c.physics.fixedStepHz);
    ki("max_sub_steps", c.physics.maxSubSteps);
    kf("time_scale", c.physics.timeScale);
    kf("max_speed_mps", c.physics.maxSpeedMps);
    kf("slip_angle_filter_hz", c.physics.slipAngleFilterHz);
    kb("apply_suspension_forces", c.physics.applySuspensionForces);
    kb("apply_chassis_torque", c.physics.applyChassisTorque);
    kb("apply_drivetrain", c.physics.applyDrivetrain);
    kb("apply_brakes", c.physics.applyBrakes);
    kf("global_grip_scale", c.physics.globalGripScale);
    kf("assist_grip_floor", c.physics.assistGripFloor);
    kb("tcs_simulation", c.physics.tcsSimulation);
    kf("tcs_slip_target", c.physics.tcsSlipTarget);
    kb("esc_simulation", c.physics.escSimulation);
    kf("esc_slip_threshold_deg", c.physics.escSlipThresholdDeg);
    kf("esc_correction_gain", c.physics.escCorrectionGain);

    section("Debug");
    kb("log_telemetry", c.debug.logTelemetry);
    kb("log_hooks", c.debug.logHooks);
    kb("dump_state_on_crash", c.debug.dumpStateOnCrash);
    kb("panic_on_guard_trip", c.debug.panicOnGuardTrip);
    ki("max_guard_trips", c.debug.maxGuardTrips);

    section("Addresses");
    d += "; Absolute addresses (hex, e.g. 0x7FF6A2C01234). Leave 0 to use signature scanning.\n";
    kv("hook_friction_matrix", HexOrZero(c.memory.pinHookFrictionMatrix));
    kv("hook_downforce", HexOrZero(c.memory.pinHookDownforce));
    kv("hook_suspension_solve", HexOrZero(c.memory.pinHookSuspension));
    kv("hook_tire_force", HexOrZero(c.memory.pinHookTireForce));
    kv("grip_scale", HexOrZero(c.memory.pinGripScale));
    kv("downforce_scale", HexOrZero(c.memory.pinDownforceScale));
    kv("brake_bias", HexOrZero(c.memory.pinBrakeBias));
    kv("steer_angle", HexOrZero(c.memory.pinSteerAngle));
    kv("friction_matrix", HexOrZero(c.memory.pinFrictionMatrix));

    section("Keys");
    ki("toggle_master", c.keys.toggleMaster);
    ki("toggle_passive", c.keys.togglePassive);
    ki("reload_config", c.keys.reloadConfig);
    ki("panic_disable", c.keys.panicDisable);
    ki("dump_state", c.keys.dumpState);

    return d;
}

} // namespace

// -----------------------------------------------------------------------------
Config& Config::Instance() noexcept
{
    static Config instance;
    return instance;
}

void Config::ApplyDefaultsToTunables() noexcept
{
    Sanitize();
}

void Config::Sanitize() noexcept
{
    vehicle.massKg              = ClampFinite(vehicle.massKg, 300.0f, 12000.0f, 1450.0f);
    vehicle.wheelbaseM          = ClampFinite(vehicle.wheelbaseM, 1.4f, 5.2f, 2.70f);
    vehicle.frontTrackM         = ClampFinite(vehicle.frontTrackM, 0.9f, 2.4f, 1.58f);
    vehicle.rearTrackM          = ClampFinite(vehicle.rearTrackM, 0.9f, 2.4f, 1.58f);
    vehicle.cgHeightM           = ClampFinite(vehicle.cgHeightM, 0.15f, 1.10f, 0.52f);
    vehicle.weightDistribution  = ClampFinite(vehicle.weightDistribution, 0.25f, 0.75f, 0.53f);
    vehicle.wheelRadiusFrontM   = ClampFinite(vehicle.wheelRadiusFrontM, 0.15f, 0.65f, 0.34f);
    vehicle.wheelRadiusRearM    = ClampFinite(vehicle.wheelRadiusRearM, 0.15f, 0.65f, 0.34f);
    vehicle.rollInertiaKgM2     = ClampFinite(vehicle.rollInertiaKgM2, 50.0f, 5000.0f, 520.0f);
    vehicle.pitchInertiaKgM2    = ClampFinite(vehicle.pitchInertiaKgM2, 200.0f, 20000.0f, 1900.0f);
    vehicle.yawInertiaKgM2      = ClampFinite(vehicle.yawInertiaKgM2, 200.0f, 20000.0f, 2300.0f);
    vehicle.rollStiffnessNmPerRad  = ClampFinite(vehicle.rollStiffnessNmPerRad, 1000.0f, 400000.0f, 34000.0f);
    vehicle.pitchStiffnessNmPerRad = ClampFinite(vehicle.pitchStiffnessNmPerRad, 1000.0f, 600000.0f, 52000.0f);
    vehicle.rollDampingNmsPerRad   = ClampFinite(vehicle.rollDampingNmsPerRad, 0.0f, 200000.0f, 5400.0f);
    vehicle.pitchDampingNmsPerRad  = ClampFinite(vehicle.pitchDampingNmsPerRad, 0.0f, 200000.0f, 7200.0f);
    vehicle.antiRollBarFrontNmPerRad = ClampFinite(vehicle.antiRollBarFrontNmPerRad, 0.0f, 200000.0f, 9000.0f);
    vehicle.antiRollBarRearNmPerRad  = ClampFinite(vehicle.antiRollBarRearNmPerRad, 0.0f, 200000.0f, 5500.0f);
    vehicle.maxSuspensionTravelM = ClampFinite(vehicle.maxSuspensionTravelM, 0.02f, 0.40f, 0.13f);
    vehicle.rideHeightM          = ClampFinite(vehicle.rideHeightM, 0.02f, 0.45f, 0.11f);
    vehicle.driveLayout          = Clamp(vehicle.driveLayout, 0, 2);

    tire.muNominalFront       = ClampFinite(tire.muNominalFront, 0.20f, 2.20f, 1.08f);
    tire.muNominalRear        = ClampFinite(tire.muNominalRear, 0.20f, 2.20f, 1.12f);
    tire.bLong                = ClampFinite(tire.bLong, 1.0f, 40.0f, 10.5f);
    tire.cLong                = ClampFinite(tire.cLong, 0.5f, 3.0f, 1.62f);
    tire.dLongScale           = ClampFinite(tire.dLongScale, 0.3f, 1.8f, 1.0f);
    tire.eLong                = ClampFinite(tire.eLong, -0.9f, 0.95f, 0.30f);
    tire.bLat                 = ClampFinite(tire.bLat, 1.0f, 40.0f, 8.40f);
    tire.cLat                 = ClampFinite(tire.cLat, 0.5f, 3.0f, 1.38f);
    tire.eLat                 = ClampFinite(tire.eLat, -0.9f, 0.95f, 0.28f);
    tire.loadSensitivity      = ClampFinite(tire.loadSensitivity, 0.0f, 0.60f, 0.165f);
    tire.loadReferenceN       = ClampFinite(tire.loadReferenceN, 300.0f, 20000.0f, 4000.0f);
    tire.camberGainPerDeg     = ClampFinite(tire.camberGainPerDeg, 0.0f, 0.05f, 0.0035f);
    tire.slipAnglePeakDeg     = ClampFinite(tire.slipAnglePeakDeg, 1.0f, 30.0f, 7.5f);
    tire.slipRatioPeak        = ClampFinite(tire.slipRatioPeak, 0.02f, 0.60f, 0.11f);
    tire.relaxationLengthM    = ClampFinite(tire.relaxationLengthM, 0.01f, 2.0f, 0.28f);
    tire.temperatureOptimumC  = ClampFinite(tire.temperatureOptimumC, 20.0f, 250.0f, 84.0f);
    tire.temperatureWindowC   = ClampFinite(tire.temperatureWindowC, 5.0f, 200.0f, 55.0f);
    tire.temperatureGripFloor = ClampFinite(tire.temperatureGripFloor, 0.30f, 1.0f, 0.72f);
    tire.wearGripFloor        = ClampFinite(tire.wearGripFloor, 0.30f, 1.0f, 0.80f);
    tire.wearRate             = ClampFinite(tire.wearRate, 0.0f, 0.01f, 0.00001f);
    tire.heatGeneration       = ClampFinite(tire.heatGeneration, 0.0f, 1.0f, 0.085f);
    tire.coolingRate          = ClampFinite(tire.coolingRate, 0.0f, 1.0f, 0.055f);

    friction.snapDeg          = ClampFinite(friction.snapDeg, 0.1f, 30.0f, 2.0f);
    friction.fadeRate         = ClampFinite(friction.fadeRate, 0.0f, 200.0f, 6.0f);
    for (int i = 0; i < 6; ++i)
    {
        friction.frontAngleKeysDeg[i] = ClampFinite(friction.frontAngleKeysDeg[i], -90.0f, 90.0f, 0.0f);
        friction.rearAngleKeysDeg[i]  = ClampFinite(friction.rearAngleKeysDeg[i], -90.0f, 90.0f, 0.0f);
        for (int j = 0; j < 4; ++j)
        {
            friction.frontValues[i][j] = ClampFinite(friction.frontValues[i][j], 0.10f, 4.0f, 1.0f);
            friction.rearValues[i][j]  = ClampFinite(friction.rearValues[i][j], 0.10f, 4.0f, 1.0f);
        }
    }
    for (int j = 0; j < 4; ++j)
        friction.loadKeysN[j] = ClampFinite(friction.loadKeysN[j], 100.0f, 40000.0f, 1000.0f * (j + 1));

    weight.longTransferGain  = ClampFinite(weight.longTransferGain, 0.0f, 2.0f, 1.0f);
    weight.latTransferGain   = ClampFinite(weight.latTransferGain, 0.0f, 2.0f, 1.0f);
    weight.pitchAccelGain    = ClampFinite(weight.pitchAccelGain, 0.0f, 3.0f, 1.0f);
    weight.rollAccelGain     = ClampFinite(weight.rollAccelGain, 0.0f, 3.0f, 1.0f);
    weight.geometryGain      = ClampFinite(weight.geometryGain, 0.0f, 1.0f, 0.35f);
    weight.jackingGain       = ClampFinite(weight.jackingGain, -1.0f, 1.0f, 0.0f);
    weight.minCornerLoadN    = ClampFinite(weight.minCornerLoadN, 0.0f, 2000.0f, 25.0f);
    weight.loadRateLimitN    = ClampFinite(weight.loadRateLimitN, 1000.0f, 1.0e7f, 400000.0f);
    weight.antiDiveGain      = ClampFinite(weight.antiDiveGain, 0.0f, 1.0f, 0.55f);
    weight.antiSquatGain     = ClampFinite(weight.antiSquatGain, 0.0f, 1.0f, 0.45f);

    aero.airDensityKgM3         = ClampFinite(aero.airDensityKgM3, 0.6f, 1.6f, 1.225f);
    aero.liftCoefficientFront   = ClampFinite(aero.liftCoefficientFront, -2.0f, 4.0f, 0.62f);
    aero.liftCoefficientRear    = ClampFinite(aero.liftCoefficientRear, -2.0f, 4.0f, 0.92f);
    aero.frontalAreaM2          = ClampFinite(aero.frontalAreaM2, 0.8f, 6.0f, 2.15f);
    aero.dragCoefficient        = ClampFinite(aero.dragCoefficient, 0.10f, 1.50f, 0.34f);
    aero.groundEffectHeightM    = ClampFinite(aero.groundEffectHeightM, 0.0f, 0.40f, 0.09f);
    aero.groundEffectGain       = ClampFinite(aero.groundEffectGain, 0.0f, 2.0f, 0.42f);
    aero.stallSpeedMps          = ClampFinite(aero.stallSpeedMps, 20.0f, 200.0f, 78.0f);
    aero.stallFalloff           = ClampFinite(aero.stallFalloff, 0.0f, 1.0f, 0.22f);
    aero.balanceSplit           = ClampFinite(aero.balanceSplit, 0.0f, 1.0f, 0.40f);
    aero.yawSensitivity         = ClampFinite(aero.yawSensitivity, -1.0f, 1.0f, 0.18f);
    aero.smoothTimeS            = ClampFinite(aero.smoothTimeS, 0.0f, 1.0f, 0.02f);
    aero.maxDownforceN          = ClampFinite(aero.maxDownforceN, 0.0f, 80000.0f, 22000.0f);
    aero.maxDragN               = ClampFinite(aero.maxDragN, 0.0f, 60000.0f, 16000.0f);

    drivetrain.torqueSplitFront     = ClampFinite(drivetrain.torqueSplitFront, 0.0f, 1.0f, 0.30f);
    drivetrain.engineBrakingGain    = ClampFinite(drivetrain.engineBrakingGain, 0.0f, 3.0f, 0.85f);
    drivetrain.clutchEngagement     = ClampFinite(drivetrain.clutchEngagement, 0.0f, 1.0f, 1.0f);
    drivetrain.drivelineInertiaKgM2 = ClampFinite(drivetrain.drivelineInertiaKgM2, 0.01f, 5.0f, 0.24f);
    drivetrain.rearBiasOnLaunch     = ClampFinite(drivetrain.rearBiasOnLaunch, 0.0f, 1.0f, 0.20f);
    drivetrain.launchSpeedMps       = ClampFinite(drivetrain.launchSpeedMps, 0.0f, 80.0f, 12.0f);
    drivetrain.maxEngineTorqueNm    = ClampFinite(drivetrain.maxEngineTorqueNm, 20.0f, 4000.0f, 620.0f);
    drivetrain.torqueCurvePeakRpm   = ClampFinite(drivetrain.torqueCurvePeakRpm, 800.0f, 20000.0f, 4600.0f);
    drivetrain.torqueCurveWidthRpm  = ClampFinite(drivetrain.torqueCurveWidthRpm, 200.0f, 10000.0f, 2100.0f);

    brakes.maxBrakeTorqueNm = ClampFinite(brakes.maxBrakeTorqueNm, 100.0f, 40000.0f, 3400.0f);
    brakes.frontBias        = ClampFinite(brakes.frontBias, 0.10f, 0.90f, 0.62f);
    brakes.absSlipThreshold = ClampFinite(brakes.absSlipThreshold, 0.02f, 0.80f, 0.22f);
    brakes.absCycleHz       = ClampFinite(brakes.absCycleHz, 1.0f, 60.0f, 12.0f);
    brakes.brakeFadeStartC  = ClampFinite(brakes.brakeFadeStartC, 100.0f, 1200.0f, 480.0f);
    brakes.brakeFadeRate    = ClampFinite(brakes.brakeFadeRate, 0.0f, 0.05f, 0.0012f);
    brakes.handbrakeTorqueNm = ClampFinite(brakes.handbrakeTorqueNm, 0.0f, 20000.0f, 2100.0f);

    steering.maxSteerAngleDeg   = ClampFinite(steering.maxSteerAngleDeg, 5.0f, 60.0f, 34.0f);
    steering.speedSensitivity   = ClampFinite(steering.speedSensitivity, 0.0f, 0.5f, 0.045f);
    steering.minAngleAtSpeedDeg = ClampFinite(steering.minAngleAtSpeedDeg, 1.0f, 40.0f, 7.5f);
    steering.ackermannGain      = ClampFinite(steering.ackermannGain, 0.0f, 1.0f, 0.65f);
    steering.counterSteerAssist = ClampFinite(steering.counterSteerAssist, 0.0f, 1.0f, 0.28f);
    steering.assistSlipDeadzoneDeg = ClampFinite(steering.assistSlipDeadzoneDeg, 0.0f, 45.0f, 3.0f);
    steering.autosteerRate      = ClampFinite(steering.autosteerRate, 0.1f, 30.0f, 3.2f);
    steering.returnRate         = ClampFinite(steering.returnRate, 0.1f, 30.0f, 4.5f);
    steering.ffbGain            = ClampFinite(steering.ffbGain, 0.0f, 3.0f, 1.0f);

    memory.resolveIntervalS   = ClampFinite(memory.resolveIntervalS, 0.1f, 60.0f, 3.0f);
    memory.scannerAlignment   = Clamp(memory.scannerAlignment, 1, 16);

    telemetry.port            = Clamp(telemetry.port, 1, 65535);
    telemetry.bindPortFallback = Clamp(telemetry.bindPortFallback, 1, 65535);
    telemetry.broadcastPort   = Clamp(telemetry.broadcastPort, 1, 65535);
    telemetry.timeoutSeconds  = ClampFinite(telemetry.timeoutSeconds, 0.2f, 60.0f, 2.5f);
    telemetry.speedFilterHz   = ClampFinite(telemetry.speedFilterHz, 1.0f, 480.0f, 22.0f);
    telemetry.maxPacketBytes  = Clamp(telemetry.maxPacketBytes, 324, 4096);

    physics.fixedStepHz       = ClampFinite(physics.fixedStepHz, 30.0f, 1000.0f, 240.0f);
    physics.maxSubSteps       = Clamp(physics.maxSubSteps, 1, 32);
    physics.timeScale         = ClampFinite(physics.timeScale, 0.05f, 4.0f, 1.0f);
    physics.maxSpeedMps       = ClampFinite(physics.maxSpeedMps, 10.0f, 300.0f, 140.0f);
    physics.slipAngleFilterHz = ClampFinite(physics.slipAngleFilterHz, 1.0f, 480.0f, 40.0f);
    physics.globalGripScale   = ClampFinite(physics.globalGripScale, 0.2f, 2.5f, 1.0f);
    physics.assistGripFloor   = ClampFinite(physics.assistGripFloor, 0.1f, 1.0f, 0.55f);
    physics.tcsSlipTarget     = ClampFinite(physics.tcsSlipTarget, 0.02f, 0.60f, 0.14f);
    physics.escSlipThresholdDeg = ClampFinite(physics.escSlipThresholdDeg, 1.0f, 45.0f, 9.0f);
    physics.escCorrectionGain = ClampFinite(physics.escCorrectionGain, 0.0f, 1.0f, 0.55f);

    debug.logLevel          = Clamp(debug.logLevel, 0, 4);
    debug.overlayMode       = Clamp(debug.overlayMode, 0, 2);
    debug.overlayIntervalS  = ClampFinite(debug.overlayIntervalS, 0.05f, 5.0f, 0.25f);
    debug.maxGuardTrips     = Clamp(debug.maxGuardTrips, 1, 1000);

    if (moduleName.empty())
        moduleName = "FH4RPO";
}

bool Config::Load(const std::string& path) noexcept
{
    iniPath = path;

    std::FILE* file = nullptr;
#ifdef _MSC_VER
    if (fopen_s(&file, path.c_str(), "rb") != 0)
        file = nullptr;
#else
    file = std::fopen(path.c_str(), "rb");
#endif
    if (file == nullptr)
    {
        FH4RPO_LOG_WARN("Config: %s not found, writing defaults", path.c_str());
        ApplyDefaultsToTunables();
        Save(path);
        return false;
    }

    std::string text;
    char chunk[4096];
    std::size_t read = 0;
    while ((read = std::fread(chunk, 1, sizeof(chunk), file)) > 0)
    {
        text.append(chunk, read);
        if (text.size() > (4 * 1024 * 1024))
        {
            std::fclose(file);
            FH4RPO_LOG_ERROR("Config: %s is unreasonably large, using defaults", path.c_str());
            ApplyDefaultsToTunables();
            return false;
        }
    }
    std::fclose(file);

    if (text.empty())
    {
        FH4RPO_LOG_ERROR("Config: %s is empty, using defaults", path.c_str());
        ApplyDefaultsToTunables();
        return false;
    }

    SectionMap map;
    if (!ParseIni(text, map))
    {
        ApplyDefaultsToTunables();
        return false;
    }

    const Reader general(map, "General");
    moduleName = general.GetString("module_name", moduleName.empty() ? "FH4RPO" : moduleName);
    debug.logLevel = general.GetInt("log_level", debug.logLevel);
    debug.overlayMode = general.GetInt("overlay_mode", debug.overlayMode);
    debug.overlayIntervalS = general.GetFloat("overlay_interval_s", debug.overlayIntervalS);

    {
        const Reader r(map, "Vehicle");
        vehicle.massKg              = r.GetFloat("mass_kg", vehicle.massKg);
        vehicle.wheelbaseM          = r.GetFloat("wheelbase_m", vehicle.wheelbaseM);
        vehicle.frontTrackM         = r.GetFloat("front_track_m", vehicle.frontTrackM);
        vehicle.rearTrackM          = r.GetFloat("rear_track_m", vehicle.rearTrackM);
        vehicle.cgHeightM           = r.GetFloat("cg_height_m", vehicle.cgHeightM);
        vehicle.weightDistribution  = r.GetFloat("weight_distribution", vehicle.weightDistribution);
        vehicle.wheelRadiusFrontM   = r.GetFloat("wheel_radius_front_m", vehicle.wheelRadiusFrontM);
        vehicle.wheelRadiusRearM    = r.GetFloat("wheel_radius_rear_m", vehicle.wheelRadiusRearM);
        vehicle.rollInertiaKgM2     = r.GetFloat("roll_inertia_kgm2", vehicle.rollInertiaKgM2);
        vehicle.pitchInertiaKgM2    = r.GetFloat("pitch_inertia_kgm2", vehicle.pitchInertiaKgM2);
        vehicle.yawInertiaKgM2      = r.GetFloat("yaw_inertia_kgm2", vehicle.yawInertiaKgM2);
        vehicle.rollStiffnessNmPerRad   = r.GetFloat("roll_stiffness_nm_per_rad", vehicle.rollStiffnessNmPerRad);
        vehicle.pitchStiffnessNmPerRad  = r.GetFloat("pitch_stiffness_nm_per_rad", vehicle.pitchStiffnessNmPerRad);
        vehicle.rollDampingNmsPerRad    = r.GetFloat("roll_damping_nms_per_rad", vehicle.rollDampingNmsPerRad);
        vehicle.pitchDampingNmsPerRad   = r.GetFloat("pitch_damping_nms_per_rad", vehicle.pitchDampingNmsPerRad);
        vehicle.antiRollBarFrontNmPerRad = r.GetFloat("anti_roll_front_nm_per_rad", vehicle.antiRollBarFrontNmPerRad);
        vehicle.antiRollBarRearNmPerRad  = r.GetFloat("anti_roll_rear_nm_per_rad", vehicle.antiRollBarRearNmPerRad);
        vehicle.maxSuspensionTravelM = r.GetFloat("max_suspension_travel_m", vehicle.maxSuspensionTravelM);
        vehicle.rideHeightM          = r.GetFloat("ride_height_m", vehicle.rideHeightM);
        vehicle.frontTireWidthM      = r.GetFloat("front_tire_width_m", vehicle.frontTireWidthM);
        vehicle.rearTireWidthM       = r.GetFloat("rear_tire_width_m", vehicle.rearTireWidthM);
        vehicle.driveLayout          = r.GetInt("drive_layout", vehicle.driveLayout);
    }

    {
        const Reader r(map, "Tires");
        tire.muNominalFront       = r.GetFloat("mu_front", tire.muNominalFront);
        tire.muNominalRear        = r.GetFloat("mu_rear", tire.muNominalRear);
        tire.bLong                = r.GetFloat("b_long", tire.bLong);
        tire.cLong                = r.GetFloat("c_long", tire.cLong);
        tire.dLongScale           = r.GetFloat("d_long_scale", tire.dLongScale);
        tire.eLong                = r.GetFloat("e_long", tire.eLong);
        tire.bLat                 = r.GetFloat("b_lat", tire.bLat);
        tire.cLat                 = r.GetFloat("c_lat", tire.cLat);
        tire.eLat                 = r.GetFloat("e_lat", tire.eLat);
        tire.loadSensitivity      = r.GetFloat("load_sensitivity", tire.loadSensitivity);
        tire.loadReferenceN       = r.GetFloat("load_reference_n", tire.loadReferenceN);
        tire.camberGainPerDeg     = r.GetFloat("camber_gain_per_deg", tire.camberGainPerDeg);
        tire.slipAnglePeakDeg     = r.GetFloat("slip_angle_peak_deg", tire.slipAnglePeakDeg);
        tire.slipRatioPeak        = r.GetFloat("slip_ratio_peak", tire.slipRatioPeak);
        tire.relaxationLengthM    = r.GetFloat("relaxation_length_m", tire.relaxationLengthM);
        tire.temperatureOptimumC  = r.GetFloat("temperature_optimum_c", tire.temperatureOptimumC);
        tire.temperatureWindowC   = r.GetFloat("temperature_window_c", tire.temperatureWindowC);
        tire.temperatureGripFloor = r.GetFloat("temperature_grip_floor", tire.temperatureGripFloor);
        tire.wearGripFloor        = r.GetFloat("wear_grip_floor", tire.wearGripFloor);
        tire.wearRate             = r.GetFloat("wear_rate", tire.wearRate);
        tire.heatGeneration       = r.GetFloat("heat_generation", tire.heatGeneration);
        tire.coolingRate          = r.GetFloat("cooling_rate", tire.coolingRate);
        tire.surfaceFrictionWet   = r.GetFloat("surface_friction_wet", tire.surfaceFrictionWet);
        tire.surfaceFrictionGrass = r.GetFloat("surface_friction_grass", tire.surfaceFrictionGrass);
        tire.surfaceFrictionDirt  = r.GetFloat("surface_friction_dirt", tire.surfaceFrictionDirt);
        tire.surfaceFrictionSnow  = r.GetFloat("surface_friction_snow", tire.surfaceFrictionSnow);
    }

    {
        const Reader r(map, "FrictionMatrix");
        friction.enabled   = r.GetBool("enabled", friction.enabled);
        friction.asymmetric = r.GetBool("asymmetric", friction.asymmetric);
        friction.perAxle   = r.GetBool("per_axle", friction.perAxle);
        friction.useLoadAxis = r.GetBool("use_load_axis", friction.useLoadAxis);
        friction.snapDeg   = r.GetFloat("snap_deg", friction.snapDeg);
        friction.fadeRate  = r.GetFloat("fade_rate", friction.fadeRate);
        r.GetFloatList("front_angle_keys_deg", friction.frontAngleKeysDeg, 6, 6);
        r.GetFloatList("rear_angle_keys_deg", friction.rearAngleKeysDeg, 6, 6);
        r.GetFloatList("load_keys_n", friction.loadKeysN, 4, 4);
        r.GetMatrix("front_values", &friction.frontValues[0][0], 6, 4);
        r.GetMatrix("rear_values", &friction.rearValues[0][0], 6, 4);
    }

    {
        const Reader r(map, "WeightTransfer");
        weight.enabled = r.GetBool("enabled", weight.enabled);
        weight.longTransferGain = r.GetFloat("long_transfer_gain", weight.longTransferGain);
        weight.latTransferGain  = r.GetFloat("lat_transfer_gain", weight.latTransferGain);
        weight.pitchAccelGain   = r.GetFloat("pitch_accel_gain", weight.pitchAccelGain);
        weight.rollAccelGain    = r.GetFloat("roll_accel_gain", weight.rollAccelGain);
        weight.geometryGain     = r.GetFloat("geometry_gain", weight.geometryGain);
        weight.jackingGain      = r.GetFloat("jacking_gain", weight.jackingGain);
        weight.minCornerLoadN   = r.GetFloat("min_corner_load_n", weight.minCornerLoadN);
        weight.loadRateLimitN   = r.GetFloat("load_rate_limit_n", weight.loadRateLimitN);
        weight.antiDiveGain     = r.GetFloat("anti_dive_gain", weight.antiDiveGain);
        weight.antiSquatGain    = r.GetFloat("anti_squat_gain", weight.antiSquatGain);
    }

    {
        const Reader r(map, "Aero");
        aero.enabled = r.GetBool("enabled", aero.enabled);
        aero.overrideGameDownforce = r.GetBool("override_game_downforce", aero.overrideGameDownforce);
        aero.airDensityKgM3       = r.GetFloat("air_density_kgm3", aero.airDensityKgM3);
        aero.liftCoefficientFront = r.GetFloat("lift_coefficient_front", aero.liftCoefficientFront);
        aero.liftCoefficientRear  = r.GetFloat("lift_coefficient_rear", aero.liftCoefficientRear);
        aero.frontalAreaM2        = r.GetFloat("frontal_area_m2", aero.frontalAreaM2);
        aero.dragCoefficient      = r.GetFloat("drag_coefficient", aero.dragCoefficient);
        aero.groundEffectHeightM  = r.GetFloat("ground_effect_height_m", aero.groundEffectHeightM);
        aero.groundEffectGain     = r.GetFloat("ground_effect_gain", aero.groundEffectGain);
        aero.stallSpeedMps        = r.GetFloat("stall_speed_mps", aero.stallSpeedMps);
        aero.stallFalloff         = r.GetFloat("stall_falloff", aero.stallFalloff);
        aero.balanceSplit         = r.GetFloat("balance_split", aero.balanceSplit);
        aero.yawSensitivity       = r.GetFloat("yaw_sensitivity", aero.yawSensitivity);
        aero.smoothTimeS          = r.GetFloat("smooth_time_s", aero.smoothTimeS);
        aero.maxDownforceN        = r.GetFloat("max_downforce_n", aero.maxDownforceN);
        aero.maxDragN             = r.GetFloat("max_drag_n", aero.maxDragN);
    }

    {
        const Reader r(map, "Drivetrain");
        drivetrain.enabled = r.GetBool("enabled", drivetrain.enabled);
        drivetrain.torqueSplitFront  = r.GetFloat("torque_split_front", drivetrain.torqueSplitFront);
        drivetrain.engineBrakingGain = r.GetFloat("engine_braking_gain", drivetrain.engineBrakingGain);
        drivetrain.clutchEngagement  = r.GetFloat("clutch_engagement", drivetrain.clutchEngagement);
        drivetrain.drivelineInertiaKgM2 = r.GetFloat("driveline_inertia_kgm2", drivetrain.drivelineInertiaKgM2);
        drivetrain.rearBiasOnLaunch  = r.GetFloat("rear_bias_on_launch", drivetrain.rearBiasOnLaunch);
        drivetrain.launchSpeedMps    = r.GetFloat("launch_speed_mps", drivetrain.launchSpeedMps);
        drivetrain.maxEngineTorqueNm = r.GetFloat("max_engine_torque_nm", drivetrain.maxEngineTorqueNm);
        drivetrain.torqueCurvePeakRpm = r.GetFloat("torque_curve_peak_rpm", drivetrain.torqueCurvePeakRpm);
        drivetrain.torqueCurveWidthRpm = r.GetFloat("torque_curve_width_rpm", drivetrain.torqueCurveWidthRpm);
    }

    {
        const Reader r(map, "Brakes");
        brakes.enabled = r.GetBool("enabled", brakes.enabled);
        brakes.maxBrakeTorqueNm = r.GetFloat("max_brake_torque_nm", brakes.maxBrakeTorqueNm);
        brakes.frontBias        = r.GetFloat("front_bias", brakes.frontBias);
        brakes.absSlipThreshold = r.GetFloat("abs_slip_threshold", brakes.absSlipThreshold);
        brakes.absCycleHz       = r.GetFloat("abs_cycle_hz", brakes.absCycleHz);
        brakes.brakeFadeStartC  = r.GetFloat("brake_fade_start_c", brakes.brakeFadeStartC);
        brakes.brakeFadeRate    = r.GetFloat("brake_fade_rate", brakes.brakeFadeRate);
        brakes.handbrakeTorqueNm = r.GetFloat("handbrake_torque_nm", brakes.handbrakeTorqueNm);
    }

    {
        const Reader r(map, "Steering");
        steering.enabled = r.GetBool("enabled", steering.enabled);
        steering.maxSteerAngleDeg   = r.GetFloat("max_steer_angle_deg", steering.maxSteerAngleDeg);
        steering.speedSensitivity   = r.GetFloat("speed_sensitivity", steering.speedSensitivity);
        steering.minAngleAtSpeedDeg = r.GetFloat("min_angle_at_speed_deg", steering.minAngleAtSpeedDeg);
        steering.ackermannGain      = r.GetFloat("ackermann_gain", steering.ackermannGain);
        steering.counterSteerAssist = r.GetFloat("counter_steer_assist", steering.counterSteerAssist);
        steering.assistSlipDeadzoneDeg = r.GetFloat("assist_slip_deadzone_deg", steering.assistSlipDeadzoneDeg);
        steering.autosteerRate      = r.GetFloat("autosteer_rate", steering.autosteerRate);
        steering.returnRate         = r.GetFloat("return_rate", steering.returnRate);
        steering.ffbGain            = r.GetFloat("ffb_gain", steering.ffbGain);
    }

    {
        const Reader r(map, "Memory");
        memory.enableWriteGuards  = r.GetBool("enable_write_guards", memory.enableWriteGuards);
        memory.enableSelfTest     = r.GetBool("enable_self_test", memory.enableSelfTest);
        memory.verifyHookTargets  = r.GetBool("verify_hook_targets", memory.verifyHookTargets);
        memory.resolveIntervalS   = r.GetFloat("resolve_interval_s", memory.resolveIntervalS);
        memory.scannerAlignment   = r.GetInt("scanner_alignment", memory.scannerAlignment);
        memory.enableDebugPrivilege = r.GetBool("enable_debug_privilege", memory.enableDebugPrivilege);
        memory.guardOnSuspend     = r.GetBool("guard_on_suspend", memory.guardOnSuspend);
    }

    {
        const Reader r(map, "Telemetry");
        telemetry.enabled          = r.GetBool("enabled", telemetry.enabled);
        telemetry.port             = r.GetInt("port", telemetry.port);
        telemetry.bindPortFallback = r.GetInt("bind_port_fallback", telemetry.bindPortFallback);
        telemetry.bindLoopbackOnly = r.GetBool("bind_loopback_only", telemetry.bindLoopbackOnly);
        telemetry.timeoutSeconds   = r.GetFloat("timeout_seconds", telemetry.timeoutSeconds);
        telemetry.speedFilterHz    = r.GetFloat("speed_filter_hz", telemetry.speedFilterHz);
        telemetry.autoDeriveSpec   = r.GetBool("auto_derive_spec", telemetry.autoDeriveSpec);
        telemetry.broadcastAssist  = r.GetBool("broadcast_assist", telemetry.broadcastAssist);
        telemetry.broadcastPort    = r.GetInt("broadcast_port", telemetry.broadcastPort);
    }

    {
        const Reader r(map, "Physics");
        physics.enabled              = r.GetBool("enabled", physics.enabled);
        physics.passiveModeAllowed   = r.GetBool("passive_mode_allowed", physics.passiveModeAllowed);
        physics.fixedStepHz          = r.GetFloat("fixed_step_hz", physics.fixedStepHz);
        physics.maxSubSteps          = r.GetInt("max_sub_steps", physics.maxSubSteps);
        physics.timeScale            = r.GetFloat("time_scale", physics.timeScale);
        physics.maxSpeedMps          = r.GetFloat("max_speed_mps", physics.maxSpeedMps);
        physics.slipAngleFilterHz    = r.GetFloat("slip_angle_filter_hz", physics.slipAngleFilterHz);
        physics.applySuspensionForces = r.GetBool("apply_suspension_forces", physics.applySuspensionForces);
        physics.applyChassisTorque   = r.GetBool("apply_chassis_torque", physics.applyChassisTorque);
        physics.applyDrivetrain      = r.GetBool("apply_drivetrain", physics.applyDrivetrain);
        physics.applyBrakes          = r.GetBool("apply_brakes", physics.applyBrakes);
        physics.globalGripScale      = r.GetFloat("global_grip_scale", physics.globalGripScale);
        physics.assistGripFloor      = r.GetFloat("assist_grip_floor", physics.assistGripFloor);
        physics.tcsSimulation        = r.GetBool("tcs_simulation", physics.tcsSimulation);
        physics.tcsSlipTarget        = r.GetFloat("tcs_slip_target", physics.tcsSlipTarget);
        physics.escSimulation        = r.GetBool("esc_simulation", physics.escSimulation);
        physics.escSlipThresholdDeg  = r.GetFloat("esc_slip_threshold_deg", physics.escSlipThresholdDeg);
        physics.escCorrectionGain    = r.GetFloat("esc_correction_gain", physics.escCorrectionGain);
    }

    {
        const Reader r(map, "Debug");
        debug.logTelemetry       = r.GetBool("log_telemetry", debug.logTelemetry);
        debug.logHooks           = r.GetBool("log_hooks", debug.logHooks);
        debug.dumpStateOnCrash   = r.GetBool("dump_state_on_crash", debug.dumpStateOnCrash);
        debug.panicOnGuardTrip   = r.GetBool("panic_on_guard_trip", debug.panicOnGuardTrip);
        debug.maxGuardTrips      = r.GetInt("max_guard_trips", debug.maxGuardTrips);
    }

    {
        const Reader r(map, "Addresses");
        memory.pinHookFrictionMatrix = ParseAddress(r.GetString("hook_friction_matrix", "0"));
        memory.pinHookDownforce      = ParseAddress(r.GetString("hook_downforce", "0"));
        memory.pinHookSuspension     = ParseAddress(r.GetString("hook_suspension_solve", "0"));
        memory.pinHookTireForce      = ParseAddress(r.GetString("hook_tire_force", "0"));
        memory.pinGripScale          = ParseAddress(r.GetString("grip_scale", "0"));
        memory.pinDownforceScale     = ParseAddress(r.GetString("downforce_scale", "0"));
        memory.pinBrakeBias          = ParseAddress(r.GetString("brake_bias", "0"));
        memory.pinSteerAngle         = ParseAddress(r.GetString("steer_angle", "0"));
        memory.pinFrictionMatrix     = ParseAddress(r.GetString("friction_matrix", "0"));
    }

    {
        const Reader r(map, "Keys");
        keys.toggleMaster  = r.GetInt("toggle_master", keys.toggleMaster);
        keys.togglePassive = r.GetInt("toggle_passive", keys.togglePassive);
        keys.reloadConfig  = r.GetInt("reload_config", keys.reloadConfig);
        keys.panicDisable  = r.GetInt("panic_disable", keys.panicDisable);
        keys.dumpState     = r.GetInt("dump_state", keys.dumpState);
    }

    Sanitize();
    return true;
}

bool Config::Save(const std::string& path) const noexcept
{
    const std::string document = BuildDefaultDocument(*this);

    std::FILE* file = nullptr;
#ifdef _MSC_VER
    if (fopen_s(&file, path.c_str(), "wb") != 0)
        file = nullptr;
#else
    file = std::fopen(path.c_str(), "wb");
#endif
    if (file == nullptr)
    {
        FH4RPO_LOG_WARN("Config: cannot write %s", path.c_str());
        return false;
    }

    const std::size_t written = std::fwrite(document.data(), 1, document.size(), file);
    std::fflush(file);
    std::fclose(file);

    if (written != document.size())
    {
        FH4RPO_LOG_WARN("Config: short write on %s", path.c_str());
        return false;
    }
    return true;
}

bool Config::HasFileChanged() const noexcept
{
    if (iniPath.empty())
        return false;

    struct stat info{};
    if (::stat(iniPath.c_str(), &info) != 0)
        return false;

    static std::int64_t lastWrite = 0;

    const std::int64_t stamp = static_cast<std::int64_t>(info.st_mtime);
    if (lastWrite == 0)
    {
        lastWrite = stamp;
        return false;
    }
    if (stamp != lastWrite)
    {
        lastWrite = stamp;
        return true;
    }
    return false;
}

void Config::WatchFile() noexcept
{
    if (!HasFileChanged())
        return;

    const std::string path = iniPath;
    if (path.empty())
        return;

    // Parse into a scratch instance first, then copy the tunables across so a
    // half-written file can never leave the live configuration inconsistent.
    reloadInProgress.store(true, std::memory_order_release);
    Config scratch;
    scratch.moduleName = moduleName;
    scratch.iniPath = path;
    if (!scratch.Load(path))
    {
        reloadInProgress.store(false, std::memory_order_release);
        FH4RPO_LOG_WARN("Config: hot reload failed, keeping previous values");
        return;
    }

    vehicle  = scratch.vehicle;
    tire     = scratch.tire;
    friction = scratch.friction;
    weight   = scratch.weight;
    aero     = scratch.aero;
    drivetrain = scratch.drivetrain;
    brakes   = scratch.brakes;
    steering = scratch.steering;
    memory   = scratch.memory;
    telemetry = scratch.telemetry;
    physics  = scratch.physics;
    debug    = scratch.debug;
    keys     = scratch.keys;
    Sanitize();
    reloadInProgress.store(false, std::memory_order_release);

    FH4RPO_LOG_INFO("Config: hot reloaded from %s", path.c_str());
}

} // namespace fh4rpo
