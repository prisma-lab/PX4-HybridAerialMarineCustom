#!/usr/bin/env python3
"""Exercise production window duration, timestamp anchoring and late-command checks."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'src/modules/marine_takeoff/MarineTakeoff.cpp').read_text()


def function(name):
    begin = source.index(f'MarineTakeoff::{name}(')
    begin = source.rfind('\n', 0, begin) + 1
    brace = source.index('{', begin)
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[begin:end]


anchor = re.search(r'_last_sample_time = synchronized.reference_index.*?;', source, re.S)[0]
origin = re.search(r'_prediction_start_time = _last_sample_time.*?;', source)[0]
code = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <limits>
using hrt_abstime = uint64_t;
static hrt_abstime clock_now = 0;
hrt_abstime hrt_absolute_time() { return clock_now; }
#define PX4_INFO(...) ((void)0)
#define PX4_ISFINITE(x) std::isfinite(x)
class MarineTakeoff {
public:
 static constexpr size_t FFT_SIZE=5000;
 float _final_takeoff_window[FFT_SIZE]{};
 struct {float value=1.f;float get() const{return value;}} _param_win_time;
 bool _is_window_duration_valid=false,_was_window_valid=false,_takeoff_triggered=false,_takeoff_completed=false;
 int _valid_window_start_idx=-1;
 hrt_abstime _last_countdown_print=0;
 hrt_abstime _last_sample_time=0,_prediction_start_time=0,_target_takeoff_time=0,_takeoff_window_end_time=0;
 void check_window_validity();
 bool takeoff_window_has_time_remaining(hrt_abstime now) const;
 void sample_anchor(int reference, hrt_abstime sample_time, hrt_abstime attitude_time, hrt_abstime imu_time) {
  struct {int reference_index;} synchronized{reference};
  struct {hrt_abstime timestamp_sample;} _last_attitude{sample_time};
  struct {hrt_abstime timestamp;} streams[]={{attitude_time},{imu_time}};
  ANCHOR
  ORIGIN
 }
 void reset_window() {
  std::fill_n(_final_takeoff_window,FFT_SIZE,0.f);
  _prediction_start_time=100000000;
  clock_now=99900000;
 }
 void good(int start,int count){std::fill_n(_final_takeoff_window+start,count,1.f);}
};
FUNCTIONS
int main(){
 MarineTakeoff m;
 m.sample_anchor(0,1234000,1250000,1260000);
 assert(m._last_sample_time==1234000 && m._prediction_start_time==1238000);
 m.sample_anchor(0,0,1250000,1260000);
 assert(m._prediction_start_time==1254000);
 m.sample_anchor(1,1234000,1250000,1260000);
 assert(m._prediction_start_time==1264000);
 m.sample_anchor(1,1234000,1250000,0);
 assert(m._prediction_start_time==0);
 m.check_window_validity();assert(!m._takeoff_triggered);
 // Exactly 1 s between first and last prediction does not satisfy >1 s.
 m.reset_window();m.good(0,251);m.check_window_validity();assert(!m._takeoff_triggered);
 m.reset_window();m.good(0,256);m.check_window_validity();assert(!m._takeoff_triggered);
 m.good(256,1);m.check_window_validity();
 assert(m._takeoff_triggered && m._target_takeoff_time==100000000);
 assert(m._takeoff_window_end_time==101024000);
 assert(m.takeoff_window_has_time_remaining(100000000));
 assert(m.takeoff_window_has_time_remaining(100019000));
 assert(!m.takeoff_window_has_time_remaining(100024000));
 assert(!m.takeoff_window_has_time_remaining(102000000));
 // Non-grid minimum must be exceeded rather than rounded down.
 m.reset_window();m._param_win_time.value=1.001f;m.good(0,251);m.check_window_validity();assert(!m._takeoff_triggered);
 m.good(251,7);m.check_window_validity();assert(m._takeoff_triggered);
 m._param_win_time.value=1.f;
 // An earlier short run must not merge with the next valid run.
 m.reset_window();m.good(0,251);m.good(300,258);m.check_window_validity();
 assert(m._valid_window_start_idx==300 && m._target_takeoff_time==101200000);
 // Runtime latency changes remaining validity, never shifts the prediction timeline.
 m.reset_window();m.good(0,1000);clock_now=100100000;m.check_window_validity();
 assert(m._valid_window_start_idx==30 && m._target_takeoff_time==100120000);
 m.reset_window();m.good(0,252);clock_now=100500000;m.check_window_validity();assert(!m._takeoff_triggered);
 // Stale or entirely expired data cannot schedule a command.
 m.reset_window();m.good(0,5000);clock_now=130000000;m.check_window_validity();assert(!m._takeoff_triggered);
 // Invalid predictions and durations fail closed.
 m.reset_window();m.good(0,251);m._final_takeoff_window[251]=std::numeric_limits<float>::quiet_NaN();
 m.check_window_validity();assert(!m._takeoff_triggered);
 m._param_win_time.value=std::numeric_limits<float>::quiet_NaN();m.check_window_validity();assert(!m._takeoff_triggered);
 m._param_win_time.value=0.f;m.check_window_validity();assert(!m._takeoff_triggered);
 m.reset_window();m.good(0,5000);m._param_win_time.value=20.f;m.check_window_validity();assert(!m._takeoff_triggered);
 puts("PASS: measurement/reference timestamps, strict endpoint duration, off-grid threshold, fragmented windows, processing delay, expiry and late-command cancellation.");
}
'''.replace('ANCHOR', anchor).replace('ORIGIN', origin).replace('FUNCTIONS', '\n'.join(function(name) for name in ('check_window_validity', 'takeoff_window_has_time_remaining')))
with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp/'test.cpp').write_text(code)
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(tmp/'test.cpp'), '-o', str(tmp/'test')], check=True)
    subprocess.run([str(tmp/'test')], check=True)
