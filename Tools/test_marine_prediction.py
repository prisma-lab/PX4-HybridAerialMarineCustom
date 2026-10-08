#!/usr/bin/env python3
"""Exercise production spectral prediction with real KISS FFT and attitude-biased windows."""
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[1]
s=(root/'src/modules/marine_takeoff/MarineTakeoff.cpp').read_text()
def function(name):
 start=s.index('void MarineTakeoff::'+name+'(')
 brace=s.index('{',start); depth=1;end=brace+1
 while depth:
  depth+=(s[end]=='{')-(s[end]=='}');end+=1
 return s[start:end]
code=r'''
#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <cassert>
#include "kiss_fftr.h"
#define M_PI_F 3.14159265358979323846f
class MarineTakeoff {
public:
 static constexpr int FFT_SIZE=5000;
 kiss_fftr_cfg _fft_cfg=kiss_fftr_alloc(FFT_SIZE,0,nullptr,nullptr);
 kiss_fftr_cfg _fft_inv_cfg=kiss_fftr_alloc(FFT_SIZE,1,nullptr,nullptr);
 kiss_fft_cpx _fft_out[2501]{},_fft_accel_copy[2501]{};
 float _pred_roll_out[FFT_SIZE]{},_pred_pitch_out[FFT_SIZE]{},_prediction_send[FFT_SIZE]{},_final_takeoff_window[FFT_SIZE]{};
 struct {float get(){return .069f;}} _param_att_threshold;
 void double_int(float*,float*,bool,bool);
 void predict_wave(float*,int,const kiss_fft_cpx*,bool=false);
 void build_prediction_window();
};
FUNCTIONS
int main(){
 MarineTakeoff m; float input[5000],output[5000],reference[5000],temporary[5000];
 const float omega=2*M_PI_F*.25f;
 for(int i=0;i<5000;i++)input[i]=-omega*omega*.2f*cosf(omega*i/250.f);
 m.double_int(input,output,true,true);
 for(int i=0;i<5000;i++)assert(fabsf(output[i]-.2f*cosf(omega*i/250.f))<1e-5f);
 for(float mean: {-.2f,.2f}) {
  for(auto &v:input)v=mean;
  kiss_fftr(m._fft_cfg,input,m._fft_out);
  m.predict_wave(m._pred_roll_out,5000,m._fft_out);
  for(auto v:m._pred_roll_out)assert(fabsf(v-mean)<1e-5f);
  for(auto &v:m._prediction_send)v=-.1f;
  m.build_prediction_window();
  for(auto v:m._final_takeoff_window)assert(v==0.f);
 }
 for(int i=0;i<5000;i++)input[i]=.2f+.04f*cosf(omega*i/250.f);
 kiss_fftr(m._fft_cfg,input,m._fft_out);
 m.predict_wave(output,5000,m._fft_out);
 for(int i=0;i<5000;i++)assert(fabsf(output[i]-(.2f+.04f*cosf(omega*(20.f+i/250.f))))<1e-4f);
 for(int i=0;i<5000;i++)input[i]=1.f-omega*omega*.2f*cosf(omega*i/250.f);
 kiss_fftr(m._fft_cfg,input,m._fft_accel_copy);
 m.predict_wave(output,5000,m._fft_accel_copy,true);
 for(int i=0;i<5000;i++)assert(fabsf(output[i]-.2f*cosf(omega*(20.f+i/250.f)))<5e-4f);
 for(auto &v:input)v=1.f;
 kiss_fftr(m._fft_cfg,input,m._fft_accel_copy);
 m.predict_wave(output,5000,m._fft_accel_copy,true);
 for(auto v:output)assert(fabsf(v)<1e-6f);
 // Compare direct spectral integration against the former time-domain round trip.
 for(int trial=0;trial<8;trial++) {
  memset(m._fft_accel_copy,0,sizeof(m._fft_accel_copy));
  m._fft_accel_copy[0].r=5000.f; // gravity/bias DC must have no effect on displacement
  for(int k=2;k<=16;k++) {
   float phase=.17f*k+.1f*trial;
   float magnitude=20.f+k+trial;
   m._fft_accel_copy[k]={magnitude*cosf(phase),magnitude*sinf(phase)};
  }
  // Keep the former selection threshold and frequency band before integration.
  m._fft_accel_copy[1].r=100.f;
  m._fft_accel_copy[20].r=100.f;
  m._fft_accel_copy[8]={.1f,0.f};
  m.predict_wave(output,5000,m._fft_accel_copy,true);
  m._fft_accel_copy[0].r=0.f;
  m.predict_wave(temporary,5000,m._fft_accel_copy,false);
  m.double_int(temporary,reference,true,false);
  float error=0;
  for(int i=0;i<5000;i++)error=fmaxf(error,fabsf(output[i]-reference[i]));
  printf("Round-trip comparison %d: max absolute error %.8f m\n",trial,double(error));
  // Allow the numerical phase drift already present in the former round trip.
  assert(error<1e-4f);
 }
 printf("PASS: direct spectral heave matches previous integration; acceleration DC excluded; signed attitude means retained; bias rejects false flat windows; oscillation plus mean preserved.\n");
}

'''.replace('FUNCTIONS','\n'.join(function(x) for x in ('double_int','predict_wave','build_prediction_window')))
import tempfile
workspace=tempfile.TemporaryDirectory()
p=Path(workspace.name);(p/'check.cpp').write_text(code)
lib=root/'src/lib/kissFFT'
for name in ('kiss_fft','kiss_fftr'):
 subprocess.run(['cc','-O2','-I',str(lib),'-c',str(lib/(name+'.c')),'-o',str(p/(name+'.o'))],check=True)
subprocess.run(['c++','-O2','-std=c++17','-I',str(lib),str(p/'check.cpp'),str(p/'kiss_fft.o'),str(p/'kiss_fftr.o'),'-o',str(p/'check')],check=True)
subprocess.run([str(p/'check')],check=True)
