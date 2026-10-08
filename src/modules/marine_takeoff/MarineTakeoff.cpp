#include "MarineTakeoff.hpp"

MarineTakeoff::MarineTakeoff() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

MarineTakeoff::~MarineTakeoff()
{
    if (_fft_cfg) { free(_fft_cfg); }
    if (_fft_inv_cfg) { free(_fft_inv_cfg); }
    if (_fft_cfg_att) { free(_fft_cfg_att); }
    if (_fft_inv_cfg_att) { free(_fft_inv_cfg_att); }
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool MarineTakeoff::init()
{
    _fft_cfg = kiss_fftr_alloc(FFT_SIZE, 0, nullptr, nullptr);
    _fft_inv_cfg = kiss_fftr_alloc(FFT_SIZE, 1, nullptr, nullptr);
    _fft_cfg_att = kiss_fftr_alloc(FFT_SIZE, 0, nullptr, nullptr);
    _fft_inv_cfg_att = kiss_fftr_alloc(FFT_SIZE, 1, nullptr, nullptr);

    if (_fft_cfg == nullptr || _fft_inv_cfg == nullptr
        || _fft_cfg_att == nullptr || _fft_inv_cfg_att == nullptr) {
        PX4_ERR("FFT allocation failed!");
        return false;
    }

    ScheduleOnInterval(4_ms); // 250Hz
    return true;
}

void MarineTakeoff::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    if (_vehicle_status_sub.updated()) {
        vehicle_status_s vehicle_status;
        if (_vehicle_status_sub.copy(&vehicle_status)) {
            _armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
        }
    }

    _vehicle_control_mode_sub.update(&_vehicle_control_mode);
    if (!_vehicle_control_mode.flag_control_marine_takeoff_enabled) {
        if (_marine_prev) {
            reset_module();
        }
        _marine_prev = false;
        perf_end(_loop_perf);
        return;
    }
    _marine_prev = true;

    if (_vehicle_control_mode.flag_control_marine_takeoff_enabled ) {
        if (!_armed) {
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0f;
        cmd.target_system = 1; cmd.target_component = 1;
        _vehicle_command_pub.publish(cmd);




     }

    }




    // TAKEOFF
 if (_takeoff_triggered && !_takeoff_completed) {
        const hrt_abstime now = hrt_absolute_time();
        if (now >= _target_takeoff_time
            && !takeoff_window_has_time_remaining(now)) {
            _takeoff_triggered = false;
            PX4_WARN("Marine Takeoff: window expired or insufficient time remaining");
        } else if (now >= _target_takeoff_time) {
             PX4_WARN("Marine Takeoff: Takeoff start !");

             vehicle_command_s mode_cmd{};
             mode_cmd.timestamp = hrt_absolute_time();
             mode_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;

             mode_cmd.param1 = 1.0f;
             mode_cmd.param2 = 4.0f;
             mode_cmd.param3 = 2.0f;

             mode_cmd.target_system = 1;
             mode_cmd.target_component = 1;
             mode_cmd.source_system = 1;
             mode_cmd.source_component = 1;
             mode_cmd.from_external = false;

             if (_vehicle_command_pub.publish(mode_cmd)) {
                 _takeoff_completed = true; // stop fft
                 //PX4_INFO("Sent TAKEOFF  command. stop FFT .");
             } else {
                 PX4_ERR(" Error takoff not sent");
             }
         }
     }


    if (!_takeoff_completed){
    const bool att_upd = _vehicle_attitude_sub.updated();
    const bool imu_upd = _vehicle_imu_status_sub.updated();

    if(_vehicle_control_mode.flag_control_marine_takeoff_enabled ){
    if (att_upd) _vehicle_attitude_sub.copy(&_last_attitude);
    if (imu_upd) _vehicle_imu_status_sub.copy(&_last_imu_status);


    zoh_utils::ZOHProcessor::Input streams[] = {
        {_last_attitude.q, _last_attitude.timestamp, sizeof(_last_attitude.q), att_upd},
        {_last_imu_status.mean_accel, _last_imu_status.timestamp, sizeof(_last_imu_status.mean_accel), imu_upd}
    };

    const auto synchronized = _zoh_processor.process(streams, 2);
    if (synchronized.updated) {
        if (_cycle_count < _max_cycles) {

            if (_total_samples == 0) {
                PX4_INFO("Sample acquisition ...");
            }

            //  Transform acceleration into the world frame
            Quatf q(_last_attitude.q);
            Vector3f acc_body(_last_imu_status.mean_accel[0], _last_imu_status.mean_accel[1], _last_imu_status.mean_accel[2]);
            Vector3f acc_world = q.rotateVector(acc_body);
            float acc_z_pure = acc_world(2) + 9.80665f;

            matrix::Eulerf euler(q);
            float current_roll = euler.phi();   // Roll rad
            float current_pitch = euler.theta(); // Pitch rad

            // ---  BUFFER ---

            const float * sources[] = { &acc_z_pure,  &current_roll, &current_pitch };
            float* targets[] = { _buf_az, _buf_roll, _buf_pitch };


            _zoh_processor.bufferDataMulti(sources, targets, 3, _fft_idx, FFT_SIZE);
            // Anchor the synchronized sample to its reference topic, not processing time.
            _last_sample_time = synchronized.reference_index == 0 && _last_attitude.timestamp_sample != 0
                                ? _last_attitude.timestamp_sample
                                : streams[synchronized.reference_index].timestamp;

            // Increment total samples only until the buffer is filled for the first time
            if (_total_samples < (int)FFT_SIZE) {
            _total_samples++;}

            _step_counter++;

            // 1. Check: the buffer is full
            if (_total_samples >= (int)FFT_SIZE) {

                    if (_step_counter >= FFT_SIZE) {
                        _current_snapshot_time= hrt_absolute_time();
                        process_fft_and_snapshot();
                        // Reset counter
                        _step_counter = 0;


                        //PX4_INFO("FFT cycle number  %d", _cycle_count);
                }
             }
        }
    }

    if (_is_sending_results) {
        SendChunks();
    }

     }
    }
    perf_end(_loop_perf);
}

void MarineTakeoff::process_fft_and_snapshot()
{
    _cycle_count++;
    // Both predictions start one nominal step after the final measured sample.
    _prediction_start_time = _last_sample_time == 0 ? 0 : _last_sample_time + 4000;
    for (size_t i = 0; i < FFT_SIZE; i++) {
        size_t circ_idx = (_fft_idx + i) % FFT_SIZE;
        work_buf[i] = _buf_az[circ_idx];

        roll_work[i] = _buf_roll[circ_idx];
        pitch_work[i] = _buf_pitch[circ_idx];
    }

     // 1. Current wave : computed from RAW data
    double_int(work_buf, _displacement_send, true, true);

    // 2. Prepare data using a HANN window for the PHASE

    for (size_t i = 0; i < FFT_SIZE; i++) {
       float hann = 0.5f * (1.0f - cosf(2.0f * M_PI_F * i / (float)(FFT_SIZE - 1)));
        windowed_buf[i] = work_buf[i] ;
       windowed_buf[i] = work_buf[i] * hann;
    }

    // Windowed FFT (used only for phase)
    kiss_fftr(_fft_cfg, windowed_buf, _fft_out);


    // Pure FFT (used for true amplitude without attenuation)
    kiss_fftr(_fft_cfg, work_buf, _fft_pure);

    // Get phase and amplitude
    for (size_t  k = 0; k < (FFT_SIZE / 2 + 1); k++) {

        float mag_pure = sqrtf(_fft_pure[k].r * _fft_pure[k].r + _fft_pure[k].i * _fft_pure[k].i);
        float phase_win = atan2f(_fft_out[k].i, _fft_out[k].r);

        _fft_accel_copy[k].r = mag_pure * cosf(phase_win);
        _fft_accel_copy[k].i = mag_pure * sinf(phase_win);
    }

    memcpy(_prediction_old, _prediction_send, sizeof(_prediction_old));

    // HEAVE MOTION  prediction
    predict_heave(_prediction_send, _fft_accel_copy);

    // ROLL  Prediction

    kiss_fftr(_fft_cfg_att, roll_work, _fft_roll);


    predict_attitude(_pred_roll_out, _fft_roll);

    //  PITCH   prediction

    kiss_fftr(_fft_cfg_att, pitch_work, _fft_pitch);


    predict_attitude(_pred_pitch_out, _fft_pitch);

     // takeoff window
    build_prediction_window();
    check_window_validity();


    // Debug
    int window_count = 0;
    for(size_t i = 0; i < FFT_SIZE; i++) {
        if(_final_takeoff_window[i] > 0.5f) window_count++;
    }
    if(window_count > 0) {
        PX4_INFO("Square Wave sample: %d good sample  %d", window_count, (int)FFT_SIZE);
}



    _is_sending_results = true;
    _current_chunk_idx = 0;
}

void MarineTakeoff::double_int(float* input_time_buf, float* output_time_buf, bool apply_integration, bool is_real_data)
{
    kiss_fftr(_fft_cfg, input_time_buf, _fft_out);



    if (is_real_data) {
        memcpy(_fft_accel_copy, _fft_out, sizeof(_fft_accel_copy));


    }

    const float Fs = 250.0f;
    const float df = Fs / (float)FFT_SIZE;

    _fft_out[0].r = 0.0f; _fft_out[0].i = 0.0f;

    for (int k = 1; k < (int)(FFT_SIZE / 2 + 1); k++) {
        float freq = k * df;
        if (freq >= 0.01f && freq <= 2.0f) {
            if (apply_integration) {
                float omega = 2.0f * M_PI_F * freq;
                float factor = -1.0f / (omega * omega);
                _fft_out[k].r *= factor;
                _fft_out[k].i *= factor;
            }
        } else {
            _fft_out[k].r = 0.0f; _fft_out[k].i = 0.0f;
        }
    }

    kiss_fftri(_fft_inv_cfg, _fft_out, output_time_buf);

    const float norm_factor = 1.0f / (float)FFT_SIZE;
    for (size_t i = 0; i < FFT_SIZE; i++) {
        output_time_buf[i] *= norm_factor;
    }
}

void MarineTakeoff::predict_attitude(float* prediction_buf, const kiss_fft_cpx* fft_source)
{
    // The future samples start at N/Fs. Every DFT harmonic therefore advances by
    // 2*pi*k: a full-spectrum IFFT is exactly this periodic continuation.
    // Preserve DC, Nyquist and all other bins, without the heave noise filters.
    kiss_fftri(_fft_inv_cfg_att, fft_source, prediction_buf);

    for (size_t i = 0; i < FFT_SIZE; ++i) {
        prediction_buf[i] /= (float)FFT_SIZE;
    }
}

void MarineTakeoff::predict_heave(float* prediction_buf, const kiss_fft_cpx* fft_source)
{
    constexpr int prediction_samples = FFT_SIZE;
    const float Fs = 250.0f;
    const float dt = 1.0f / Fs;
    const float df = Fs / (float)FFT_SIZE;
    const float norm = 2.0f / (float)FFT_SIZE;
    const float T_to_last_sample = (float)(FFT_SIZE - 1) * dt;

    // Acceleration DC is excluded from integrated displacement.
    std::fill_n(prediction_buf, prediction_samples, 0.0f);

    for (int k = 1; k < (int)(FFT_SIZE / 2); k++) {
        float freq = k * df;

        if (freq >= 0.1f && freq <= 0.8f) {


            float real = fft_source[k].r;
            float imag = fft_source[k].i;
            float mag_sq = (real * real + imag * imag);

            if (mag_sq > 0.1f) {
                float amplitude = sqrtf(mag_sq) * norm;
                float omega = 2.0f * M_PI_F * freq;

                // H(k) = A(k) / (j * omega)^2: reconstruct future displacement directly.
                amplitude *= -1.0f / (omega * omega);

                float phase_start = atan2f(imag, real);
                float phase_now = phase_start + (omega * T_to_last_sample);

                float phase_step = omega * dt;
                float current_phase = phase_now + phase_step;

                for (int t_idx = 0; t_idx < prediction_samples; t_idx++) {

                    prediction_buf[t_idx] += amplitude * cosf(current_phase);
                    current_phase += phase_step;
                }
            }
        }
    }
}


void MarineTakeoff::reset_module()
{
    _takeoff_triggered = false;
    _takeoff_completed = false;
    _total_samples = 0;
    _cycle_count = 0;
    _is_sending_results = false;
    _was_window_valid = false;
    _fft_idx = 0;
    _step_counter = FFT_SIZE;
    _zoh_processor.reset();
    _is_window_duration_valid = false;
    _valid_window_start_idx = -1;
    _target_takeoff_time = 0;
    _last_sample_time = 0;
    _prediction_start_time = 0;
    _takeoff_window_end_time = 0;
    _snapshot_timestamp = 0;
    _current_snapshot_time = 0;
    _current_chunk_idx = 0;

    // clear buffer
    memset(_buf_az, 0, sizeof(_buf_az));
    memset(_buf_roll, 0, sizeof(_buf_roll));
    memset(_buf_pitch, 0, sizeof(_buf_pitch));

    PX4_INFO("reset module.Ready again .");
}
void MarineTakeoff::SendChunks()
{
    static hrt_abstime last_send_time = 0;
    hrt_abstime now = hrt_absolute_time();

    if (now - last_send_time < 12000) return;

    if (_current_chunk_idx < (int)(FFT_SIZE / 50)) {
        int base = _current_chunk_idx * 50;

        for (int i = 0; i < 50; i++) {
            _log_msg.wave_amplitude[i] = 0;
            _log_msg.displacement_z[i] = _displacement_send[base + i];
            _pred_msg.wave_prediction[i] = _prediction_old[base + i];
            _window_msg.takeoff_window[i] = _final_takeoff_window[base + i];

        }

        _log_msg.timestamp =   _current_snapshot_time;
        _log_msg.cycle_count = _cycle_count;
        _log_msg.sample_idx = _current_chunk_idx;
        _marine_wave_result_pub.publish(_log_msg);

        _pred_msg.timestamp =   _current_snapshot_time;
        _pred_msg.cycle_count = _cycle_count;
        _pred_msg.sample_idx = _current_chunk_idx+100;
        _marine_prediction_pub.publish(_pred_msg);


        _window_msg.timestamp =   _current_snapshot_time;
        _window_msg.cycle_count = _cycle_count;
        _window_msg.sample_idx = _current_chunk_idx+100;
        _marine_window_pub.publish(_window_msg);

        last_send_time = now;
        _current_chunk_idx++;
    } else {
        _is_sending_results = false;
        _current_chunk_idx = 0;
    }
}


void MarineTakeoff::build_prediction_window()
{
    // set threshold
    const float attitude_threshold = _param_att_threshold.get();;


    for (size_t i = 0; i < FFT_SIZE; i++) {

        //  FLAT DETECTION
        bool is_flat_predicted = (fabsf(_pred_roll_out[i]) < attitude_threshold) &&
                                 (fabsf(_pred_pitch_out[i]) < attitude_threshold);

       // WAVE CREST DETECTION
        bool is_acc_negative = (_prediction_send[i] < 0.0f);


        if (is_flat_predicted && is_acc_negative) {
            _final_takeoff_window[i] = 1.0f;
        } else {
            _final_takeoff_window[i] = 0.0f;
        }
    }
}


bool MarineTakeoff::takeoff_window_has_time_remaining(hrt_abstime now) const
{
    const float win_time = _param_win_time.get();
    return PX4_ISFINITE(win_time) && win_time > 0.0f && _takeoff_window_end_time > now
           && (double)(_takeoff_window_end_time - now) > (double)win_time * 1e6;
}

void MarineTakeoff::check_window_validity()
{
    constexpr hrt_abstime prediction_step_us = 4000;
    const float win_time = _param_win_time.get();
    const hrt_abstime now = hrt_absolute_time();

    _is_window_duration_valid = false;
    _valid_window_start_idx = -1;
    _takeoff_triggered = false;
    _target_takeoff_time = 0;
    _takeoff_window_end_time = 0;

    if (_prediction_start_time == 0 || !PX4_ISFINITE(win_time) || win_time <= 0.0f) {
        _was_window_valid = false;
        return;
    }

    int run_start = -1;

    for (size_t i = 0; i < FFT_SIZE; ++i) {
        const hrt_abstime sample_time = _prediction_start_time + i * prediction_step_us;

        // Discard elapsed predictions; late processing must not schedule a past window.
        if (sample_time <= now || !PX4_ISFINITE(_final_takeoff_window[i])
            || _final_takeoff_window[i] < 0.5f) {
            run_start = -1;
            continue;
        }

        if (run_start < 0) {
            run_start = (int)i;
        }

        const hrt_abstime start_time = _prediction_start_time + run_start * prediction_step_us;
        const double duration_us = (double)(sample_time - start_time);

        // Use endpoint times and the strict thesis condition T_w > T_min.
        if (duration_us > (double)win_time * 1e6) {
            _is_window_duration_valid = true;
            _valid_window_start_idx = run_start;
            _target_takeoff_time = start_time;

            // Retain the full endpoint of this favorable run for late-command validation.
            size_t end = i;

            while (end + 1 < FFT_SIZE && PX4_ISFINITE(_final_takeoff_window[end + 1])
                   && _final_takeoff_window[end + 1] >= 0.5f) {
                ++end;
            }

            _takeoff_window_end_time = _prediction_start_time + end * prediction_step_us;
            _takeoff_triggered = !_takeoff_completed;
            PX4_INFO("Marine Takeoff: window starts in %.3f s (sample index: %d)",
                     (double)(_target_takeoff_time - now) / 1e6, run_start);
            break;
        }
    }

    _was_window_valid = _is_window_duration_valid;
}

int MarineTakeoff::task_spawn(int argc, char *argv[])
{
    MarineTakeoff *instance = new MarineTakeoff();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("failed allocation ");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int MarineTakeoff::print_status()
{
    PX4_INFO("FFT completed: %d", _cycle_count);
    PX4_INFO("current sample : %d / %d", _total_samples, (int)FFT_SIZE);

    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
    return 0;
}

int MarineTakeoff::custom_command(int argc, char *argv[])
{
    return print_usage("command unknown");
}

int MarineTakeoff::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Descrizione
Heave motion and takeoff module .
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("marine_takeoff", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int marine_takeoff_main(int argc, char *argv[])
{
    return MarineTakeoff::main(argc, argv);
}
