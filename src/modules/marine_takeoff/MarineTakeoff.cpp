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

            if (_takeoff_command_sent && !_takeoff_completed && vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF) {
                _takeoff_completed = true;
                PX4_INFO("Marine Takeoff: AUTO TAKEOFF active; aerial controllers executing maneuver");
            }
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
    if (!_marine_prev) {
        PX4_INFO("Marine Takeoff: recording cycle 1/%d started (%d samples; nominal 20 s)",
                 _max_cycles, (int)FFT_SIZE);
        PX4_INFO("Marine Takeoff: attitude limit %.2f deg; required favorable duration > %.3f s",
                 (double)_param_att_threshold.get() * 180.0 / (double)M_PI_F, (double)_param_win_time.get());
    }

    _marine_prev = true;
    const hrt_abstime now = hrt_absolute_time();
    report_progress(now);

    if (!_armed && (_last_arm_request == 0 || now - _last_arm_request >= 1000000)) {
        vehicle_command_s cmd{};
        cmd.timestamp = now;
        cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0f;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        const bool published = _vehicle_command_pub.publish(cmd);
        _last_arm_request = now;

        if (published) {
            PX4_INFO("Marine Takeoff: waiting for arming; ARM request sent");

        } else {
            PX4_ERR("Marine Takeoff: failed to publish ARM request");
        }
    }

    update_takeoff_command(now);

    if (!_takeoff_completed && !_takeoff_command_sent) {
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
    PX4_INFO("Marine Takeoff: recording complete; calculating prediction %d/%d", _cycle_count, _max_cycles);
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


    PX4_INFO("Marine Takeoff: prediction processing %.1f ms; sample-to-decision age %.1f ms",
             (double)(hrt_absolute_time() - _current_snapshot_time) / 1000.0,
             _last_sample_time > 0 && hrt_absolute_time() >= _last_sample_time
             ? (double)(hrt_absolute_time() - _last_sample_time) / 1000.0 : 0.0);

    if (!_takeoff_triggered) {
        size_t flat_count = 0, heave_count = 0, combined_count = 0;
        size_t consecutive = 0, longest = 0;

        for (size_t i = 0; i < FFT_SIZE; ++i) {
            flat_count += fabsf(_pred_roll_out[i]) < _param_att_threshold.get()
                          && fabsf(_pred_pitch_out[i]) < _param_att_threshold.get();
            heave_count += _prediction_send[i] < 0.0f;
            const bool good = _final_takeoff_window[i] > 0.5f;
            combined_count += good;
            consecutive = good ? consecutive + 1 : 0;
            longest = std::max(longest, consecutive);
        }

        PX4_WARN("Marine Takeoff: prediction %d/%d found NO schedulable takeoff instant", _cycle_count, _max_cycles);
        PX4_INFO("Marine Takeoff: mean roll %.2f deg, mean pitch %.2f deg",
                 (double)(_fft_roll[0].r / (float)FFT_SIZE) * 180.0 / (double)M_PI_F,
                 (double)(_fft_pitch[0].r / (float)FFT_SIZE) * 180.0 / (double)M_PI_F);
        PX4_INFO("Marine Takeoff: flat %d/%d, favorable heave %d/%d, combined %d/%d",
                 (int)flat_count, (int)FFT_SIZE, (int)heave_count, (int)FFT_SIZE,
                 (int)combined_count, (int)FFT_SIZE);
        PX4_INFO("Marine Takeoff: longest combined interval %.3f s; required > %.3f s plus scheduling margin",
                 longest > 0 ? (double)(longest - 1) * 0.004 : 0.0, (double)_param_win_time.get());
    }

    if (_cycle_count < _max_cycles) {
        PX4_INFO("Marine Takeoff: continuing recording cycle %d/%d", _cycle_count + 1, _max_cycles);
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
    _takeoff_command_sent = false;
    _takeoff_acknowledged = false;
    _max_cycles_reported = false;
    _takeoff_command_timestamp = 0;
    _last_progress_print = 0;
    _last_countdown_print = 0;
    _last_arm_request = 0;
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

    PX4_INFO("Marine Takeoff: mode exited; recording and scheduled command cleared");
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


void MarineTakeoff::report_progress(hrt_abstime now)
{
    if (_cycle_count >= _max_cycles && !_takeoff_triggered && !_takeoff_command_sent) {
        if (!_max_cycles_reported) {
            PX4_WARN("Marine Takeoff: all %d prediction cycles exhausted; NO takeoff scheduled", _max_cycles);
            PX4_WARN("Marine Takeoff: leave and re-enter the mode to record a new attempt");
            _max_cycles_reported = true;
        }

        return;
    }

    if (!_takeoff_command_sent && _cycle_count < _max_cycles
        && (_last_progress_print == 0 || now - _last_progress_print >= 2000000)) {
        const int samples = _total_samples < (int)FFT_SIZE ? _total_samples : (int)_step_counter;
        PX4_INFO("Marine Takeoff: recording cycle %d/%d: %d/%d samples (%.1f%%)",
                 _cycle_count + 1, _max_cycles, samples, (int)FFT_SIZE,
                 100.0 * (double)samples / (double)FFT_SIZE);
        if (_last_sample_time == 0) {
            PX4_WARN("Marine Takeoff: waiting for attitude/IMU updates; no samples recorded yet");

        } else if (now > _last_sample_time && now - _last_sample_time > 1000000) {
            PX4_WARN("Marine Takeoff: latest sample is %.2f s old; check attitude/IMU topic updates",
                     (double)(now - _last_sample_time) / 1e6);
        }

        _last_progress_print = now;
    }
}

void MarineTakeoff::update_takeoff_command(hrt_abstime now)
{
    vehicle_command_ack_s ack{};

    while (_vehicle_command_ack_sub.update(&ack)) {
        if (ack.target_system != 1 || ack.target_component != 1) {
            continue;
        }

        if (ack.command == vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM
            && ack.timestamp >= _last_arm_request && _last_arm_request != 0
            && ack.result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
            && ack.result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
            PX4_WARN("Marine Takeoff: ARM request rejected (result %u)", (unsigned)ack.result);
        }

        if (!_takeoff_command_sent || ack.command != vehicle_command_s::VEHICLE_CMD_DO_SET_MODE
            || ack.timestamp < _takeoff_command_timestamp) {
            continue;
        }

        if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
            || ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
            if (!_takeoff_acknowledged) {
                PX4_INFO("Marine Takeoff: takeoff request acknowledged (result %u); awaiting AUTO TAKEOFF state",
                         (unsigned)ack.result);
                _takeoff_acknowledged = true;
            }

        } else {
            PX4_ERR("Marine Takeoff: AUTO TAKEOFF request rejected (result %u); no transition confirmed",
                    (unsigned)ack.result);
            _takeoff_command_sent = false;
            _takeoff_triggered = false;
        }
    }

    if (!_takeoff_triggered || _takeoff_completed) {
        return;
    }

    if (_takeoff_command_sent) {
        if (now - _takeoff_command_timestamp >= 2000000) {
            PX4_ERR("Marine Takeoff: AUTO TAKEOFF transition not confirmed after 2 s");
            _takeoff_command_sent = false;
            _takeoff_triggered = false;
        }

        return;
    }

    if (now < _target_takeoff_time) {
        if (_last_countdown_print == 0 || now - _last_countdown_print >= 1000000) {
            PX4_INFO("Marine Takeoff: takeoff command in %.2f s",
                     (double)(_target_takeoff_time - now) / 1e6);
            _last_countdown_print = now;
        }

        return;
    }

    if (!_armed) {
        PX4_WARN("Marine Takeoff: scheduled command canceled; vehicle is not armed");
        _takeoff_triggered = false;
        return;
    }

    if (!takeoff_window_has_time_remaining(now)) {
        PX4_WARN("Marine Takeoff: scheduled command canceled; favorable interval too short or expired");
        _takeoff_triggered = false;
        return;
    }

    vehicle_command_s cmd{};
    cmd.timestamp = now;
    cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1.0f;
    cmd.param2 = 4.0f;
    cmd.param3 = 2.0f;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = false;

    if (_vehicle_command_pub.publish(cmd)) {
        _takeoff_command_sent = true;
        _takeoff_acknowledged = false;
        _takeoff_command_timestamp = now;
        PX4_INFO("Marine Takeoff: takeoff command SENT; awaiting acceptance and mode transition");

    } else {
        PX4_ERR("Marine Takeoff: failed to publish takeoff command");
        _takeoff_triggered = false;
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
    // Reserve five work-queue periods for logging and normal dispatch jitter.
    constexpr hrt_abstime scheduling_margin_us = 20000;
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
        if (sample_time < now + scheduling_margin_us || !PX4_ISFINITE(_final_takeoff_window[i])
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
        if (duration_us > (double)win_time * 1e6 + scheduling_margin_us) {
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
            _last_countdown_print = now;
            PX4_INFO("Marine Takeoff: prediction SUCCESS; takeoff command in %.3f s",
                     (double)(_target_takeoff_time - now) / 1e6);
            PX4_INFO("Marine Takeoff: scheduled favorable interval %.3f s; required > %.3f s",
                     (double)(_takeoff_window_end_time - _target_takeoff_time) / 1e6, (double)win_time);
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
    PX4_INFO("Marine Takeoff: active %s; armed %s; predictions %d/%d",
             _marine_prev ? "yes" : "no", _armed ? "yes" : "no", _cycle_count, _max_cycles);
    PX4_INFO("Marine Takeoff: recorded samples %d/%d; command pending %s; AUTO TAKEOFF confirmed %s",
             _total_samples, (int)FFT_SIZE, _takeoff_command_sent ? "yes" : "no", _takeoff_completed ? "yes" : "no");

    if (_takeoff_triggered && !_takeoff_command_sent && !_takeoff_completed) {
        const hrt_abstime now = hrt_absolute_time();
        PX4_INFO("Marine Takeoff: takeoff command in %.3f s",
                 _target_takeoff_time > now ? (double)(_target_takeoff_time - now) / 1e6 : 0.0);
    }

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
