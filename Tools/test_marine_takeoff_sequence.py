#!/usr/bin/env python3
"""Execute the production FFT-to-command sequence with synthetic motion and mocked uORB."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT/'src/modules/marine_takeoff/MarineTakeoff.cpp').read_text()

def extract(name):
    start = source.index('MarineTakeoff::'+name+'(')
    start = source.rfind('\n',0,start)+1
    brace = source.index('{',start)
    end,depth = brace+1,1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}')
        end += 1
    return source[start:end]

names = ['process_fft_and_snapshot','double_int','predict_attitude','predict_heave','build_prediction_window',
         'check_window_validity','takeoff_window_has_time_remaining','report_progress','update_takeoff_command']
start=source.index('if (_takeoff_command_sent && !_takeoff_completed && vehicle_status.nav_state')
brace=source.index('{',start)
end=source.index('\n            }',brace)+len('\n            }')
confirmation=source[start:end]
code=r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>
#include "kiss_fftr.h"
using hrt_abstime=uint64_t;
static hrt_abstime clock_now=100000000;
hrt_abstime hrt_absolute_time(){return clock_now;}
static std::vector<std::string> logs;
template<typename... Args> void log_message(const char *fmt, Args... args) {
 char text[512];std::snprintf(text,sizeof(text),fmt,args...);logs.emplace_back(text);
}
#define PX4_INFO(...) log_message(__VA_ARGS__)
#define PX4_WARN(...) log_message(__VA_ARGS__)
#define PX4_ERR(...) log_message(__VA_ARGS__)
#define PX4_ISFINITE(x) std::isfinite(x)
#define M_PI_F 3.14159265358979323846f
struct vehicle_command_s {
 static constexpr unsigned VEHICLE_CMD_COMPONENT_ARM_DISARM=400, VEHICLE_CMD_DO_SET_MODE=176;
 uint64_t timestamp=0;unsigned command=0;float param1=0,param2=0,param3=0;
 int target_system=0,target_component=0,source_system=0,source_component=0;bool from_external=false;
};
struct vehicle_command_ack_s {
 static constexpr unsigned VEHICLE_CMD_RESULT_ACCEPTED=0,VEHICLE_CMD_RESULT_IN_PROGRESS=5;
 uint64_t timestamp=0;unsigned command=0,result=0;int target_system=1,target_component=1;
};
struct vehicle_status_s {static constexpr uint8_t NAVIGATION_STATE_AUTO_TAKEOFF=17;uint8_t nav_state;};
struct Parameter {float value;float get() const{return value;}};
class MarineTakeoff {
public:
 static constexpr int FFT_SIZE=5000;
 Parameter _param_att_threshold{.069f},_param_win_time{.8f};
 float _buf_az[5000]{},_buf_roll[5000]{},_buf_pitch[5000]{},work_buf[5000]{},roll_work[5000]{},pitch_work[5000]{};
 float _displacement_send[5000]{},windowed_buf[5000]{},_prediction_old[5000]{},_prediction_send[5000]{};
 float _pred_roll_out[5000]{},_pred_pitch_out[5000]{},_final_takeoff_window[5000]{};
 kiss_fftr_cfg _fft_cfg=kiss_fftr_alloc(5000,0,nullptr,nullptr),_fft_inv_cfg=kiss_fftr_alloc(5000,1,nullptr,nullptr);
 kiss_fftr_cfg _fft_cfg_att=kiss_fftr_alloc(5000,0,nullptr,nullptr),_fft_inv_cfg_att=kiss_fftr_alloc(5000,1,nullptr,nullptr);
 kiss_fft_cpx _fft_out[2501]{},_fft_pure[2501]{},_fft_accel_copy[2501]{},_fft_roll[2501]{},_fft_pitch[2501]{};
 int _cycle_count=0,_max_cycles=3,_fft_idx=0,_total_samples=0,_step_counter=0,_current_chunk_idx=0,_valid_window_start_idx=-1;
 bool _takeoff_triggered=false,_takeoff_completed=false,_takeoff_command_sent=false,_takeoff_acknowledged=false;
 bool _is_window_duration_valid=false,_was_window_valid=false,_is_sending_results=false,_max_cycles_reported=false,_armed=true;
 hrt_abstime _current_snapshot_time=clock_now;
 hrt_abstime _last_sample_time=0,_prediction_start_time=0,_target_takeoff_time=0,_takeoff_window_end_time=0;
 hrt_abstime _last_countdown_print=0,_last_progress_print=0,_takeoff_command_timestamp=0,_last_arm_request=0;
 struct Publication {
  bool success=true;std::vector<vehicle_command_s> sent;
  bool publish(const vehicle_command_s &cmd){if(success)sent.push_back(cmd);return success;}
 } _vehicle_command_pub;
 struct Subscription {
  std::deque<vehicle_command_ack_s> pending;
  bool update(vehicle_command_ack_s *ack){if(pending.empty())return false;*ack=pending.front();pending.pop_front();return true;}
 } _vehicle_command_ack_sub;
 ~MarineTakeoff(){free(_fft_cfg);free(_fft_inv_cfg);free(_fft_cfg_att);free(_fft_inv_cfg_att);}
 void process_fft_and_snapshot();void double_int(float*,float*,bool,bool);
 void predict_attitude(float*,const kiss_fft_cpx*);void predict_heave(float*,const kiss_fft_cpx*);
 void build_prediction_window();void check_window_validity();
 bool takeoff_window_has_time_remaining(hrt_abstime) const;
 void report_progress(hrt_abstime);void update_takeoff_command(hrt_abstime);
 void confirm_mode(uint8_t state){vehicle_status_s vehicle_status{state};CONFIRMATION}
 void motion(float roll_bias=0.f){
  _last_sample_time=clock_now-4000;
  for(int i=0;i<5000;i++){
   const float omega=2*M_PI_F*.25f;
   _buf_az[i]=-omega*omega*.2f*cosf(omega*i/250.f);
   _buf_roll[i]=roll_bias+.03f*cosf(omega*i/250.f);
   _buf_pitch[i]=.02f*sinf(omega*i/250.f);
  }
 }
 void arm_schedule(){
  _takeoff_triggered=true;_takeoff_completed=false;_takeoff_command_sent=false;
  _takeoff_acknowledged=false;_last_countdown_print=0;_target_takeoff_time=clock_now+3000000;
  _takeoff_window_end_time=_target_takeoff_time+2000000;
 }
 void ack(unsigned result){_vehicle_command_ack_sub.pending.push_back({clock_now,176,result,1,1});}
};
FUNCTIONS
bool printed(const char *text){for(auto &line:logs)if(line.find(text)!=std::string::npos)return true;return false;}
int main(){
 MarineTakeoff m;
 m.report_progress(clock_now);assert(printed("recording cycle") && printed("no samples recorded yet"));
 assert(logs.size()==2);m.report_progress(clock_now+1000000);assert(logs.size()==2);
 logs.clear();m.motion();m.process_fft_and_snapshot();
 assert(m._takeoff_triggered && printed("prediction SUCCESS") && printed("takeoff command in"));
 assert(m._target_takeoff_time>clock_now);
 m.update_takeoff_command(clock_now);assert(m._vehicle_command_pub.sent.empty());
 clock_now=m._target_takeoff_time+4000; // Normal one-period work-queue delay.
 m.update_takeoff_command(clock_now);
 assert(m._takeoff_command_sent && !m._takeoff_completed && m._vehicle_command_pub.sent.size()==1);
 assert(printed("command SENT"));
 assert(m._vehicle_command_pub.sent.back().param2==4.f && m._vehicle_command_pub.sent.back().param3==2.f);
 m.ack(0);m.update_takeoff_command(clock_now+4000);assert(m._takeoff_acknowledged && !m._takeoff_completed);
 assert(m._vehicle_command_pub.sent.size()==1);
 m.confirm_mode(17);assert(m._takeoff_completed && printed("AUTO TAKEOFF active"));
 logs.clear();m._takeoff_completed=false;m._takeoff_command_sent=false;m.motion(.2f);m.process_fft_and_snapshot();
 assert(!m._takeoff_triggered && printed("NO schedulable") && printed("mean roll"));
 logs.clear();m.arm_schedule();m.update_takeoff_command(clock_now);
 assert(printed("takeoff command in") && logs.size()==1);
 m.update_takeoff_command(clock_now+500000);assert(logs.size()==1);
 m.update_takeoff_command(clock_now+1000000);assert(logs.size()==2);
 clock_now=m._target_takeoff_time;m.update_takeoff_command(clock_now);
 m.ack(2);m.update_takeoff_command(clock_now+4000);
 assert(!m._takeoff_command_sent && !m._takeoff_triggered && !m._takeoff_completed && printed("request rejected"));
 m.arm_schedule();clock_now=m._target_takeoff_time;m.update_takeoff_command(clock_now);
 m.update_takeoff_command(clock_now+2000000);assert(!m._takeoff_triggered && printed("not confirmed"));
 m.arm_schedule();m._armed=false;clock_now=m._target_takeoff_time;m.update_takeoff_command(clock_now);
 assert(!m._takeoff_triggered && printed("not armed"));m._armed=true;
 m.arm_schedule();clock_now=m._takeoff_window_end_time;m.update_takeoff_command(clock_now);
 assert(!m._takeoff_triggered && printed("too short or expired"));
 m.arm_schedule();m._vehicle_command_pub.success=false;clock_now=m._target_takeoff_time;m.update_takeoff_command(clock_now);
 assert(!m._takeoff_triggered && printed("failed to publish"));
 logs.clear();m._cycle_count=3;m._takeoff_command_sent=false;m.report_progress(clock_now);m.report_progress(clock_now+1000000);
 assert(printed("cycles exhausted") && logs.size()==2);
 puts("PASS: production FFT-to-command sequence, recording/success/countdown logs, delayed dispatch, ACK versus actual mode, rejection, timeout, disarmed/expired cancellation, publication failure and cycle exhaustion.");
}
'''.replace('FUNCTIONS','\n'.join(extract(name) for name in names)).replace('CONFIRMATION',confirmation)
with tempfile.TemporaryDirectory() as directory:
    tmp=Path(directory);(tmp/'test.cpp').write_text(code)
    lib=ROOT/'src/lib/kissFFT'
    for name in ('kiss_fft','kiss_fftr'):
        subprocess.run(['cc','-O2','-I',str(lib),'-c',str(lib/(name+'.c')),'-o',str(tmp/(name+'.o'))],check=True)
    subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(lib),str(tmp/'test.cpp'),str(tmp/'kiss_fft.o'),str(tmp/'kiss_fftr.o'),'-o',str(tmp/'test')],check=True)
    subprocess.run([str(tmp/'test')],check=True)
