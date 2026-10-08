#!/usr/bin/env python3
"""Compile the production marine admission rule and entry guard; exercise all modes."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
commander = ROOT / 'src/modules/commander'
source = (commander / 'UserModeIntention.cpp').read_text()
guard = source.split('\t// Apply the same admission rule to every command path, including forced requests.\n', 1)[1].split('\t// Always allow mode change', 1)[0]
constants = re.findall(r'uint8 (NAVIGATION_STATE_\w+) = (\d+)', (ROOT / 'msg/VehicleStatus.msg').read_text())
with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp / 'uORB/topics').mkdir(parents=True)
    (tmp / 'uORB/topics/vehicle_status.h').write_text('#pragma once\n#include <cstdint>\nstruct vehicle_status_s {\n' + ''.join(f'static constexpr uint8_t {name} = {value};\n' for name, value in constants) + 'uint8_t nav_state; };\n')
    code = r'''
#include "MarineModeAdmission.hpp"
#include <cassert>
#include <cstdio>
struct vehicle_land_detected_s { bool landed = false; bool maybe_landed = true; };
struct Subject {
    vehicle_status_s _vehicle_status{};
    bool armed = false;
    struct Subscription {
        bool available = true, landed = false;
        bool copy(vehicle_land_detected_s *data) { data->landed = landed; return available; }
    } _land_detected_sub;
    bool isArmed() { return armed; }
    bool request(uint8_t user_intended_nav_state, bool force) {
        GUARD
        return true;
    }
};
int main() {
    unsigned cases = 0;
    for (uint8_t current = 0; current < vehicle_status_s::NAVIGATION_STATE_MAX; ++current) {
        for (uint8_t target = 0; target < vehicle_status_s::NAVIGATION_STATE_MAX; ++target) {
            for (unsigned bits = 0; bits < 16; ++bits) {
                Subject subject;
                subject._vehicle_status.nav_state = current;
                subject.armed = bits & 1;
                subject._land_detected_sub.landed = bits & 2;
                subject._land_detected_sub.available = bits & 4;
                const auto grouped = [](uint8_t mode) { return mode == 8 || mode == 24 || mode == 26 || mode == 27; };
                const bool expected = !grouped(target) || grouped(current)
                    || (!subject.armed && subject._land_detected_sub.available && subject._land_detected_sub.landed);
                assert(subject.request(target, bits & 8) == expected);
                ++cases;
            }
        }
    }
    printf("PASS: %u admission cases, including missing landing data, forced requests, Auto Marine and all internal transitions.\n", cases);
}
'''.replace('GUARD', guard)
    (tmp / 'test.cpp').write_text(code)
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-I', str(tmp), '-I', str(commander), str(tmp / 'test.cpp'), '-o', str(tmp / 'test')], check=True)
    subprocess.run([str(tmp / 'test')], check=True)
