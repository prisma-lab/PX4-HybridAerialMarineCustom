#!/usr/bin/env python3
"""Compare retained PRISMA mode behavior with the exact firmware1.15mods base.

Run after `make px4_sitl_default`: python3 Tools/test_hybrid_merge.py
The test compiles the original and merged mode implementations side by side.
It leaves source files and build outputs unchanged.
"""
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

BASE = "9747009bc09f27500c937760e18efd88e3544f4d"
THESIS = "de92ad0c34c35dd1b413b60d24303688fdc44b4c"
ANCESTOR = "2ae652a202dfd257d26e4bab9f94ab173ef9fe25"
ROOT = Path(__file__).resolve().parents[1]


def original(path):
    return subprocess.check_output(
        ["git", "show", f"{BASE}:{path}"], cwd=ROOT, text=True
    )


def main():
    def changed(ref):
        return set(subprocess.check_output(
            ["git", "diff", "--name-only", ANCESTOR, ref], cwd=ROOT, text=True
        ).splitlines())

    protected = changed(BASE) - changed(THESIS)
    protected.update([
        "Tools/simulation/gz/models/USV_test/model.sdf",
        "Tools/simulation/gz/worlds/WaterTakeOff.sdf",
        "ROMFS/px4fmu_common/init.d-posix/airframes/4010_gz_USV_test",
        "src/modules/commander/ModeManagement.cpp",
        "src/modules/commander/ModeManagement.hpp",
    ])
    for path in sorted(protected):
        expected = subprocess.check_output(["git", "show", f"{BASE}:{path}"], cwd=ROOT)
        if path == "src/modules/marine_navigation/MarineNavigation_quat.cpp":
            # Explicitly authorized FF and TS fixes; other original code stays identical.
            expected = expected.replace(
                b"\t\tif(fabs(rc_input.throttle) < 0.09f) control_input(0) = 0.0f;\n"
                b"\t\tif(fabs(rc_input.pitch) < 0.09f) control_input(1) = 0.0f;\n"
                b"\t\telse control_input = Vector2f(rc_input.throttle, rc_input.pitch);",
                b"\t\tcontrol_input(0) = fabsf(rc_input.throttle) < 0.09f ? 0.0f : rc_input.throttle;\n"
                b"\t\tcontrol_input(1) = fabsf(rc_input.pitch) < 0.09f ? 0.0f : rc_input.pitch;",
            )
            expected = expected.replace(
                b"control_input(1) = -(thrust +1)*steering +thrust;",
                b"control_input(1) = thrust * (1.0f - 2.0f * steering);",
            ).replace(
                b"control_input(0) = (thrust +1)*steering +thrust;",
                b"control_input(0) = thrust * (1.0f + 2.0f * steering);",
            )
        if (ROOT / path).read_bytes() != expected:
            raise AssertionError(f"Original implementation changed: {path}")
    print(f"PASS: {len(protected)} original files preserved, allowing only the authorized FF and TS fixes.", flush=True)

    commands = json.loads((ROOT / "build/px4_sitl_default/compile_commands.json").read_text())
    command = next(item for item in commands if item["file"].endswith("ModeUtil/control_mode.cpp"))
    tokens = shlex.split(command["command"])
    # Reuse the real target's include directories and platform definitions.
    flags = []
    for index, token in enumerate(tokens):
        if token.startswith(("-I", "-D")):
            flags.append(token)
        elif token == "-isystem":
            flags.extend([token, tokens[index + 1]])
    flags.extend(["-UNDEBUG", "-D__EXPORT=", "-DDEFINE_GET_PX4_CUSTOM_MODE", "-I" + str(ROOT / "src/modules/commander/ModeUtil")])
    fields = re.findall(r"^bool\s+(\w+)", original("msg/VehicleControlMode.msg"), re.M)
    field_checks = "\n".join(f"assert(before.{field} == after.{field});" for field in fields)
    constants = re.findall(r"^uint8 (NAVIGATION_STATE_\w+) = (\d+)", original("msg/VehicleStatus.msg"), re.M)
    constant_checks = "\n".join(f"static_assert(vehicle_status_s::{name} == {value});" for name, value in constants)
    custom_header = original("src/modules/commander/px4_custom_mode.h")
    active_header = re.sub(r"//[^\n]*|/\*.*?\*/", "", custom_header, flags=re.S)
    enum_names = sorted(set(re.findall(r"\bPX4_CUSTOM_(?:MAIN|SUB)_MODE_\w+", active_header)) - set(re.findall(r"enum\s+(\w+)", active_header)))
    enum_checks = "\n".join(f"static_assert(int({name}) == int(baseline_custom::{name}));" for name in enum_names)
    with tempfile.TemporaryDirectory(prefix="px4-hybrid-") as temporary:
        directory = Path(temporary)
        (directory / "base_control.cpp").write_text(original("src/modules/commander/ModeUtil/control_mode.cpp"))
        (directory / "base_custom.h").write_text(custom_header)
        (directory / "base_ui.hpp").write_text(original("src/lib/modes/ui.hpp"))
        harness = f'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <uORB/topics/vehicle_status.h>
#include "{ROOT}/src/modules/commander/ModeUtil/control_mode.cpp"
#include "{ROOT}/src/modules/commander/px4_custom_mode.h"
#include "{ROOT}/src/lib/modes/ui.hpp"
#define mode_util baseline_mode_util
#include "{directory}/base_control.cpp"
#include "{directory}/base_ui.hpp"
#undef mode_util
namespace baseline_custom {{
#include "{directory}/base_custom.h"
}}
{constant_checks}
{enum_checks}
int main() {{
    const auto new_mode = vehicle_status_s::NAVIGATION_STATE_MARINE_TAKEOFF;
    assert(new_mode == 8);
    assert(PX4_CUSTOM_SUB_MODE_MARINE_TAKEOFF == 5);
    assert(mode_util::getValidNavStates() ==
           (baseline_mode_util::getValidNavStates() | (1u << new_mode)));
    unsigned comparisons = 0;
    for (unsigned mode = 0; mode < vehicle_status_s::NAVIGATION_STATE_MAX; ++mode) {{
        if (mode == new_mode) {{ continue; }}
        assert(get_px4_custom_mode(mode).data == baseline_custom::get_px4_custom_mode(mode).data);
        assert(std::strcmp(mode_util::nav_state_names[mode], baseline_mode_util::nav_state_names[mode]) == 0);
        for (unsigned type = 0; type <= 3; ++type) {{
            for (unsigned mask = 0; mask < 128; ++mask) {{
                offboard_control_mode_s offboard{{}};
                offboard.position = mask & 1;
                offboard.velocity = mask & 2;
                offboard.acceleration = mask & 4;
                offboard.attitude = mask & 8;
                offboard.body_rate = mask & 16;
                offboard.thrust_and_torque = mask & 32;
                offboard.direct_actuator = mask & 64;
                vehicle_control_mode_s before, after;
                std::memset(&before, 0, sizeof(before));
                std::memset(&after, 0, sizeof(after));
                baseline_mode_util::getVehicleControlMode(mode, type, offboard, before);
                mode_util::getVehicleControlMode(mode, type, offboard, after);
                {field_checks}
                assert(!after.flag_control_marine_takeoff_enabled);
                ++comparisons;
            }}
        }}
    }}
    vehicle_control_mode_s marine{{}};
    offboard_control_mode_s offboard{{}};
    mode_util::getVehicleControlMode(new_mode, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, offboard, marine);
    assert(marine.flag_control_marine_takeoff_enabled && marine.flag_control_allocation_enabled);
    assert(!marine.flag_control_prisma_marine_manual_enabled);
    assert(!marine.flag_control_prisma_auto_marine_enabled);
    assert(!marine.flag_control_position_enabled && !marine.flag_control_rates_enabled);
    const auto custom = get_px4_custom_mode(new_mode);
    assert(custom.main_mode == PX4_CUSTOM_MAIN_MODE_PRISMA && custom.sub_mode == 5);
    std::printf("PASS: %u original-mode control comparisons; original IDs, MAVLink codes and names preserved; new mode distinct.\\n", comparisons);
}}
'''
        source = directory / "compare.cpp"
        source.write_text(harness)
        executable = directory / "compare"
        subprocess.run([tokens[0], "-std=c++17", *flags, str(source), "-o", str(executable)], check=True, cwd=command["directory"])
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
