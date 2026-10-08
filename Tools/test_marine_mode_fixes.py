#!/usr/bin/env python3
"""Execute the actual FF, takeoff mode-exit and sample-buffer code in isolation.

Run: python3 Tools/test_marine_mode_fixes.py
Extracted production fragments use the real ZOH implementation; no PX4 service
or simulator is required. This does not validate a complete takeoff.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    navigation = (ROOT / "src/modules/marine_navigation/MarineNavigation_quat.cpp").read_text()
    takeoff = (ROOT / "src/modules/marine_takeoff/MarineTakeoff.cpp").read_text()
    ff = navigation.split("else if (vehicle_control_mode.flag_control_prisma_marine_manual_ff_enabled) {", 1)[1].split("// UNCOMMENT", 1)[0]
    ts = navigation.split("else if (vehicle_control_mode.flag_control_prisma_marine_manual_ts_enabled) {", 1)[1].split("// UNCOMMENT", 1)[0]
    gate = takeoff.split("    _vehicle_control_mode_sub.update(&_vehicle_control_mode);", 1)[1].split("    if (!_marine_prev) {", 1)[0] + "    _marine_prev = true;"
    reset = takeoff.split("void MarineTakeoff::reset_module()", 1)[1].split("void MarineTakeoff::SendChunks()", 1)[0]
    sources = re.search(r"const float \* sources\[\].*?;", takeoff)[0]
    targets = re.search(r"float\* targets\[\].*?;", takeoff)[0]
    buffer_call = re.search(r"_zoh_processor.bufferDataMulti\(.*?;", takeoff)[0]
    size = re.search(r"FFT_SIZE = (\d+)", (ROOT / "src/modules/marine_takeoff/MarineTakeoff.hpp").read_text())[1]
    code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "ZohUtils.hpp"
#define PX4_INFO(...) ((void)0)
static unsigned perf_ends = 0;
void perf_end(int) { ++perf_ends; }
struct Vector2f {
    float values[2];
    Vector2f(float left, float right): values{left,right} {}
    float &operator()(int index) { return values[index]; }
};
struct Input { float throttle, pitch; float roll = 0; };
void ff(Input rc_input, Vector2f &control_input) { FF_CODE }
void ts(Input rc_input, Vector2f &control_input) { __TS_FRAGMENT__ }
struct MarineTakeoff {
    static constexpr int FFT_SIZE = FFT_SIZE_VALUE;
    struct { bool flag_control_marine_takeoff_enabled = false; } _vehicle_control_mode;
    int _loop_perf = 0;
    bool _marine_prev = false, _takeoff_triggered = false, _takeoff_completed = false;
    bool _is_sending_results = false, _was_window_valid = false, _is_window_duration_valid = false;
    int _total_samples = 0, _cycle_count = 0, _fft_idx = 0, _current_chunk_idx = 0, _valid_window_start_idx = -1;
    size_t _step_counter = FFT_SIZE;
    bool _takeoff_command_sent = false, _takeoff_acknowledged = false, _max_cycles_reported = false;
    uint64_t _takeoff_command_timestamp = 0, _last_progress_print = 0, _last_countdown_print = 0, _last_arm_request = 0;
    uint64_t _last_sample_time = 0, _prediction_start_time = 0, _takeoff_window_end_time = 0;
    uint64_t _target_takeoff_time = 0, _snapshot_timestamp = 0, _current_snapshot_time = 0;
    float _buf_az[FFT_SIZE]{}, _buf_roll[FFT_SIZE]{}, _buf_pitch[FFT_SIZE]{}, _raw_wave_buf[FFT_SIZE]{};
    zoh_utils::ZOHProcessor _zoh_processor;
    void reset_module();
    void gate() { GATE_CODE }
    void sample(float acc_z_pure, float current_roll, float current_pitch) {
        SOURCES_CODE
        TARGETS_CODE
        BUFFER_CODE
    }
};
void MarineTakeoff::reset_module() RESET_CODE
int main() {
    unsigned ts_cases = 0;
    for (float throttle : {-1.f, -.5f, -.09f, -.05f, 0.f, .05f, .09f, .5f, 1.f}) {
        for (float steering : {-1.f, -.5f, -.09f, -.05f, 0.f, .05f, .09f, .5f, 1.f}) {
            Vector2f result(.7f, -.7f);
            ts({throttle,0,steering}, result);
            const float t = std::fabs(throttle) < .09f ? 0.f : throttle;
            const float s = std::fabs(steering) < .09f ? 0.f : steering;
            const float left = s >= 0 ? t : t * (1 + 2*s);
            const float right = s <= 0 ? t : t * (1 - 2*s);
            assert(std::fabs(result(0) - left) < 1e-6f);
            assert(std::fabs(result(1) - right) < 1e-6f);
            ++ts_cases;
        }
    }
    std::printf("PASS: %u TS equation cases including reverse, zero throttle, full steering and deadband boundaries.\n", ts_cases);
    unsigned cases = 0;
    for (float left : {-.8f, -.05f, 0.f, .05f, .5f, .8f}) {
        for (float right : {-.8f, -.05f, 0.f, .05f, .5f, .8f}) {
            Vector2f output(.7f, -.7f);
            ff({left,right}, output);
            assert(output(0) == (std::fabs(left) < .09f ? 0.f : left));
            assert(output(1) == (std::fabs(right) < .09f ? 0.f : right));
            ++cases;
        }
    }
    Vector2f output(0,0);
    ff({.5f,0}, output); assert(output(0) == .5f && output(1) == 0);
    ff({.8f,0}, output); assert(output(0) == .8f);
    ff({0,0}, output); assert(output(0) == 0 && output(1) == 0);

    MarineTakeoff module;
    module._vehicle_control_mode.flag_control_marine_takeoff_enabled = true;
    module.gate(); assert(module._marine_prev);
    module.sample(.9f,.3f,-.4f);
    assert(module._buf_az[0] == .9f && module._buf_roll[0] == .3f && module._buf_pitch[0] == -.4f);
    assert(module._raw_wave_buf[0] == 0);
    module._total_samples = 400; module._cycle_count = 1; module._step_counter = 300;
    module._last_sample_time = 1000; module._prediction_start_time = 5000;
    module._takeoff_window_end_time = 20000;
    module._target_takeoff_time = 9000; module._current_snapshot_time = 123;
    module._valid_window_start_idx = 50; module._is_window_duration_valid = true;
    float held_value = 7;
    zoh_utils::ZOHProcessor::Input stream{&held_value,1000,sizeof(held_value),true};
    module._zoh_processor.process(&stream,1);
    // Exit during acquisition, before any takeoff is triggered.
    module._vehicle_control_mode.flag_control_marine_takeoff_enabled = false;
    module.gate();
    assert(!module._marine_prev && perf_ends == 1);
    assert(module._total_samples == 0 && module._cycle_count == 0 && module._fft_idx == 0);
    assert(module._step_counter == module.FFT_SIZE && module._target_takeoff_time == 0);
    assert(module._last_sample_time == 0 && module._prediction_start_time == 0 && module._takeoff_window_end_time == 0);
    assert(!module._is_window_duration_valid && module._valid_window_start_idx == -1);
    for (int i = 0; i < module.FFT_SIZE; ++i) {
        assert(module._buf_az[i] == 0 && module._buf_roll[i] == 0 && module._buf_pitch[i] == 0);
    }
    stream.updated = false;
    const auto held = module._zoh_processor.process(&stream,1);
    float reset_value = 1;
    std::memcpy(&reset_value, held.outputs[0], sizeof(reset_value));
    assert(reset_value == 0);
    // A pending takeoff is also cancelled, and re-entry starts fresh.
    module._vehicle_control_mode.flag_control_marine_takeoff_enabled = true;
    module.gate(); module._takeoff_triggered = true; module._is_sending_results = true;
    module._vehicle_control_mode.flag_control_marine_takeoff_enabled = false;
    module.gate();
    assert(!module._takeoff_triggered && !module._is_sending_results);
    module._vehicle_control_mode.flag_control_marine_takeoff_enabled = true;
    module.gate(); assert(module._marine_prev && module._total_samples == 0);
    std::printf("PASS: %u FF axis cases, centered-pitch updates, roll/pitch mapping, acquisition exit, pending cancellation and re-entry.\n",cases);
}
'''
    for key, value in {"FF_CODE": ff, "__TS_FRAGMENT__": ts, "GATE_CODE": gate, "RESET_CODE": reset,
                       "SOURCES_CODE": sources, "TARGETS_CODE": targets,
                       "BUFFER_CODE": buffer_call, "FFT_SIZE_VALUE": size}.items():
        code = code.replace(key, value)
    with tempfile.TemporaryDirectory(prefix="marine-fixes-") as temporary:
        source = Path(temporary) / "test.cpp"
        executable = Path(temporary) / "test"
        source.write_text(code)
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I" + str(ROOT / "src/lib/zoh_utils"),
                        str(source), str(ROOT / "src/lib/zoh_utils/ZohUtils.cpp"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
