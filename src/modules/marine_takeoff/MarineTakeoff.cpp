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
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool MarineTakeoff::init()
{
    _fft_cfg = kiss_fftr_alloc(FFT_SIZE, 0, nullptr, nullptr);
    _fft_inv_cfg = kiss_fftr_alloc(FFT_SIZE, 1, nullptr, nullptr);

    if (_fft_cfg == nullptr || _fft_inv_cfg == nullptr) {
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

    // Auto-Arming per ambiente simulato
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

            // Trasformazione accelerazione in frame mondo
            Quatf q(_last_attitude.q);
            Vector3f acc_body(_last_imu_status.mean_accel[0], _last_imu_status.mean_accel[1], _last_imu_status.mean_accel[2]);
            Vector3f acc_world = q.rotateVector(acc_body);
            float acc_z_pure = acc_world(2) + 9.80665f;

            const float * sources[] = { &acc_z_pure, &_last_wave.wave_amplitude };
            float* targets[] = { _buf_az, _raw_wave_buf };

            _zoh_processor.bufferDataMulti(sources, targets, 2, _fft_idx, FFT_SIZE);

            // Incrementiamo i campioni totali solo fino a riempire il buffer la prima volta
            if (_total_samples < (int)FFT_SIZE) {
            _total_samples++;}

            static size_t step_counter = FFT_SIZE;
            step_counter++;

            // 1. Controllo: il buffer è pieno per la prima volta?
            if (_total_samples >= (int)FFT_SIZE) {
                // 2. Controllo: sono passati 5 secondi dall'ultima esecuzione?
                    if (step_counter >= FFT_SIZE) {
                    // Calcola tutto
                        process_fft_and_snapshot();
                        // Resetta il contatore: la prossima avverrà tra esattamente 1250 campioni
                        step_counter = 0;


                        PX4_INFO("FFT eseguita al ciclo %d", _cycle_count);
                }
             }
        }
    }

    if (_is_sending_results) {
        SendChunks();
    }

    perf_end(_loop_perf);
}

void MarineTakeoff::process_fft_and_snapshot()
{

    _cycle_count++;
    // 1. Estrazione snapshot (Passato)
    for (size_t i = 0; i < FFT_SIZE; i++) {
        size_t circ_idx = (_fft_idx + i) % FFT_SIZE;
        work_buf[i] = _buf_az[circ_idx];
        _final_wave_gt_send[i] = _raw_wave_buf[circ_idx];
    }

    // 2. Calcolo Heave Passato (Backup coefficienti accelerazione)
    double_int(work_buf, _displacement_send, true, true);


    memcpy(_prediction_old, _prediction_send, sizeof(_prediction_old));
    // 3. Genera Predizione Accelerazione futura nel work_buf
    predict_wave(work_buf, FFT_SIZE);

    // 4. Integra la predizione per ottenere Heave futuro
    double_int(work_buf, _prediction_send, true, false);
    // --- AGGIUNGI QUESTE STAMPE DI DEBUG ---


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

void MarineTakeoff::predict_wave(float* prediction_acc_buf, int prediction_samples)
{
    const float Fs = 250.0f;
    const float dt = 1.0f / Fs;
    const float df = Fs / (float)FFT_SIZE;
    const float norm = 2.0f / (float)FFT_SIZE;

    // T_buf è il tempo trascorso dal PRIMO campione del buffer all'ULTIMO.
    // Per N campioni, gli intervalli sono N-1.
    const float T_to_last_sample = (float)(FFT_SIZE - 1) * dt;

    memset(prediction_acc_buf, 0, prediction_samples * sizeof(float));

    for (int k = 1; k < (int)(FFT_SIZE / 2); k++) {
        float freq = k * df;

        // Restringiamo il range alle frequenze tipiche delle onde marine (0.05 - 0.6 Hz)
        // 1.2 Hz è già un'onda molto corta/rumorosa per un drone.
        if (freq >= 0.1f && freq <= 0.8f) {

            float real = _fft_accel_copy[k].r;
            float imag = _fft_accel_copy[k].i;
            float mag_sq = (real * real + imag * imag);

            // Alziamo leggermente la soglia per ignorare il rumore bianco dell'IMU
            if (mag_sq > 0.1f) {
                float amplitude = sqrtf(mag_sq) * norm;
                float omega = 2.0f * M_PI_F * freq;

                // Calcoliamo la fase iniziale
                float phase_start = atan2f(imag, real);

                // Calcoliamo la fase esattamente all'ultimo campione registrato
                float phase_now = phase_start + (omega * T_to_last_sample);

                // Ottimizzazione: pre-calcoliamo i valori costanti per questo k
                for (int t_idx = 0; t_idx < prediction_samples; t_idx++) {
                    // Prediciamo partendo dal campione t+1 (il primo futuro)
                    float t_future = (float)(t_idx + 1) * dt;
                    prediction_acc_buf[t_idx] += amplitude * cosf(omega * t_future + phase_now);
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
