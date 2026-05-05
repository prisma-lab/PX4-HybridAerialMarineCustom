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

    // Auto-Arming
    if (!_armed) {
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0f;
        cmd.target_system = 1; cmd.target_component = 1;
        _vehicle_command_pub.publish(cmd);
    }

    const bool att_upd = _vehicle_attitude_sub.updated();
    const bool imu_upd = _vehicle_imu_status_sub.updated();

    //_vehicle_control_mode_sub.update(&_vehicle_control_mode);
    //if(_vehicle_control_mode.flag_control_marine_takeoff_enabled){
    if (att_upd) _vehicle_attitude_sub.copy(&_last_attitude);
    if (imu_upd) _vehicle_imu_status_sub.copy(&_last_imu_status);
    if (_marine_wave_sub.updated()) _marine_wave_sub.copy(&_last_wave);

    zoh_utils::ZOHProcessor::Input streams[] = {
        {_last_attitude.q, _last_attitude.timestamp, sizeof(_last_attitude.q), att_upd},
        {_last_imu_status.mean_accel, _last_imu_status.timestamp, sizeof(_last_imu_status.mean_accel), imu_upd}
    };

    if (_zoh_processor.process(streams, 2).updated) {
        if (_cycle_count < _max_cycles) {

            if (_total_samples == 0) {
                PX4_INFO("Inizio raccolta campioni...");
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

            const float * sources[] = { &acc_z_pure, &_last_wave.wave_amplitude, &current_roll, &current_pitch };
            float* targets[] = { _buf_az, _raw_wave_buf, _buf_roll, _buf_pitch };


            _zoh_processor.bufferDataMulti(sources, targets, 4, _fft_idx, FFT_SIZE);

            // Increment total samples only until the buffer is filled for the first time
            if (_total_samples < (int)FFT_SIZE) {
            _total_samples++;}

            static size_t step_counter = FFT_SIZE;
            step_counter++;

            // 1. Check: the buffer is full
            if (_total_samples >= (int)FFT_SIZE) {

                    if (step_counter >= FFT_SIZE) {

                        process_fft_and_snapshot();
                        // Reset counter
                        step_counter = 0;


                        //PX4_INFO("FFT eseguita al ciclo %d", _cycle_count);
                }
             }
        }
    }

    if (_is_sending_results) {
        SendChunks();
    }

    //  }
    perf_end(_loop_perf);
}

void MarineTakeoff::process_fft_and_snapshot()
{
    _cycle_count++;
    for (size_t i = 0; i < FFT_SIZE; i++) {
        size_t circ_idx = (_fft_idx + i) % FFT_SIZE;
        work_buf[i] = _buf_az[circ_idx];
        _final_wave_gt_send[i] = _raw_wave_buf[circ_idx];
        roll_work[i] = _buf_roll[circ_idx];
        pitch_work[i] = _buf_pitch[circ_idx];
    }

     // 1. Current wave : computed from RAW data
    double_int(work_buf, _displacement_send, true, true);

    // 2. Prepare data using a HANN window for the PHASE

    for (size_t i = 0; i < FFT_SIZE; i++) {
        float hann = 0.5f * (1.0f - cosf(2.0f * M_PI_F * i / (float)(FFT_SIZE - 1)));
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

    // 4. Predict wave
    predict_wave(work_buf, FFT_SIZE,_fft_accel_copy);
    double_int(work_buf, _prediction_send, true, false);

    // --- PREDIZIONE ROLL ---
    // "Carichiamo" la FFT del Roll nel vettore generico
    kiss_fftr(_fft_cfg_att, roll_work, _fft_roll);

    // Chiamata per il Roll (legge dal vettore generico)
    predict_wave(_pred_roll_out, FFT_SIZE,_fft_roll);

    // --- PREDIZIONE PITCH ---
    // "Carichiamo" la FFT del Pitch nel vettore generico
    kiss_fftr(_fft_cfg_att, pitch_work, _fft_pitch);

    // Chiamata per il Pitch (legge dal vettore generico)
    predict_wave(_pred_pitch_out, FFT_SIZE,_fft_pitch);

    // --- STAMPA A VIDEO DI DEBUG (Ogni 100 campioni) ---
    PX4_INFO("--- Analisi Predizione Assetto ---");
    for (size_t i = 0; i < FFT_SIZE; i += 100) {
        PX4_INFO("Campione %4d -> Roll: %6.2f deg | Pitch: %6.2f deg",
            (int)i,
            (double)math::degrees(_pred_roll_out[i]),
            (double)math::degrees(_pred_pitch_out[i]));
    }
    PX4_INFO("----------------------------------");





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
        if (freq >= 0.05f && freq <= 1.5f) {
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

void MarineTakeoff::predict_wave(float* prediction_buf, int prediction_samples, kiss_fft_cpx* fft_source)
{
    const float Fs = 250.0f;
    const float dt = 1.0f / Fs;
    const float df = Fs / (float)FFT_SIZE;
    const float norm = 2.0f / (float)FFT_SIZE;
    const float T_to_last_sample = (float)(FFT_SIZE - 1) * dt;

    memset(prediction_buf, 0, prediction_samples * sizeof(float));

    for (int k = 1; k < (int)(FFT_SIZE / 2); k++) {
        float freq = k * df;

        if (freq >= 0.1f && freq <= 0.8f) {


            float real = fft_source[k].r;
            float imag = fft_source[k].i;
            float mag_sq = (real * real + imag * imag);

            if (mag_sq > 0.1f) {
                float amplitude = sqrtf(mag_sq) * norm;
                float omega = 2.0f * M_PI_F * freq;

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

void MarineTakeoff::SendChunks()
{
    static hrt_abstime last_send_time = 0;
    hrt_abstime now = hrt_absolute_time();

    if (now - last_send_time < 12000) return;

    if (_current_chunk_idx < (int)(FFT_SIZE / 50)) {
        int base = _current_chunk_idx * 50;

        for (int i = 0; i < 50; i++) {
            _log_msg.wave_amplitude[i] = _final_wave_gt_send[base + i];
            _log_msg.displacement_z[i] = _displacement_send[base + i];
            _pred_msg.wave_prediction[i] = _prediction_old[base + i];;

        }

        _log_msg.timestamp = now;
        _log_msg.cycle_count = _cycle_count;
        _log_msg.sample_idx = _current_chunk_idx;
        _marine_wave_result_pub.publish(_log_msg);

        _pred_msg.timestamp = now;
        _pred_msg.cycle_count = _cycle_count;
        _pred_msg.sample_idx = _current_chunk_idx+100;
        _marine_prediction_pub.publish(_pred_msg);

        last_send_time = now;
        _current_chunk_idx++;
    } else {
        _is_sending_results = false;
        _current_chunk_idx = 0;
    }
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
        PX4_ERR("Allocazione fallita");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int MarineTakeoff::print_status()
{
    PX4_INFO("Cicli FFT completati: %d", _cycle_count);
    PX4_INFO("Campioni correnti: %d / %d", _total_samples, (int)FFT_SIZE);

    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
    return 0;
}

int MarineTakeoff::custom_command(int argc, char *argv[])
{
    return print_usage("Comando sconosciuto");
}

int MarineTakeoff::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Descrizione
Modulo per analisi e predizione del moto ondoso (Heave).
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
