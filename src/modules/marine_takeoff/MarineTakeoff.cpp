

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

    ScheduleOnInterval(4_ms);
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

    // 1. STATO VEICOLO
    if (_vehicle_status_sub.updated()) {
        vehicle_status_s status;
        if (_vehicle_status_sub.copy(&status)) {
            _armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
        }
    }

    // 2. ACQUISIZIONE
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

        if (_cycle_count < _max_cycles ) {

            if (_total_samples == 0) {
                vehicle_command_s start_msg{};
                start_msg.command = 888;
                _vehicle_command_pub.publish(start_msg);
                PX4_INFO("Inizio raccolta campioni...");
            }

            Quatf q(_last_attitude.q);
            Vector3f acc_body(_last_imu_status.mean_accel[0], _last_imu_status.mean_accel[1], _last_imu_status.mean_accel[2]);
            Vector3f acc_world = q.rotateVector(acc_body);
            float acc_z_pure = acc_world(2) + 9.80665f;

            _buf_az[_fft_idx] = acc_z_pure;
            _raw_wave_buf[_fft_idx] = _last_wave.wave_amplitude;

            _fft_idx = (_fft_idx + 1) % FFT_SIZE;
            _total_samples++;

            if (_total_samples >= (int)FFT_SIZE) {
                process_fft_and_snapshot();
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


    for (size_t i = 0; i < FFT_SIZE; i++) {
        size_t circ_idx = (_fft_idx + i) % FFT_SIZE;
        work_buf[i] = _buf_az[circ_idx];
        _final_wave_gt_send[i] = _raw_wave_buf[circ_idx];
    }

    kiss_fftr(_fft_cfg, work_buf, _fft_out);

    _fft_out[0].r = 0.0f; _fft_out[0].i = 0.0f;
    const float df = 250.0f / (float)FFT_SIZE;

    for (int k = 1; k < (int)(FFT_SIZE / 2 + 1); k++) {
        float freq = k * df;
        if (freq >= 0.05f && freq <= 5.0f) {
            float omega = 2.0f * M_PI_F * freq;
            float factor = -1.0f / (omega * omega);
            _fft_out[k].r *= factor;
            _fft_out[k].i *= factor;
        } else {
            _fft_out[k].r = 0.0f; _fft_out[k].i = 0.0f;
        }
    }

    kiss_fftri(_fft_inv_cfg, _fft_out, _displacement_output);

    const float norm_factor = -1.0f / (float)FFT_SIZE;
    for (size_t i = 0; i < FFT_SIZE; i++) {
        _displacement_send[i] = _displacement_output[i] * norm_factor;
    }

    _is_sending_results = true;
    _current_chunk_idx = 0;
    _cycle_count++;
    _total_samples = 0;

    vehicle_command_s save_msg{};
    save_msg.command = 999;
    _vehicle_command_pub.publish(save_msg);
    PX4_INFO("FFT completata. Invio dati...");
}

void MarineTakeoff::SendChunks()
{
    // 1. CONTROLLO TEMPORALE (PACING)
    // Impedisce l'invio di più di un chunk ogni 8 millisecondi
    static hrt_abstime last_send_time = 0;
    hrt_abstime now = hrt_absolute_time();

    if (now - last_send_time < 12000) {
        return; // Troppo presto, esce e riprova al prossimo ciclo di Run()
    }

    // 2. LOGICA DI INVIO
    if (_current_chunk_idx < (int)(FFT_SIZE / 50)) {

        int base = _current_chunk_idx * 50;

        for (int i = 0; i < 50; i++) {
            _log_msg.wave_amplitude[i] = _final_wave_gt_send[base + i];
            _log_msg.displacement_z[i] = _displacement_send[base + i];
        }

        _log_msg.timestamp = now; // Usa il tempo corrente
        _log_msg.cycle_count = _cycle_count - 1;
        _log_msg.sample_idx = _current_chunk_idx;

        // Pubblica il messaggio
        _marine_wave_result_pub.publish(_log_msg);

        // Aggiorna il timestamp dell'ultimo invio riuscito
        last_send_time = now;

        // Incrementa l'indice per il prossimo pacchetto
        _current_chunk_idx++;

    } else {
        // Fine dello streaming
        _is_sending_results = false;
        _current_chunk_idx = 0;
        PX4_INFO("Streaming completato senza perdite.");
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
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int MarineTakeoff::print_status()
{
    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
    return 0;
}

int MarineTakeoff::custom_command(int argc, char *argv[])
{

    return print_usage("UNKNOWN COMMAND ! Use: start_log o stop_log");

}

int MarineTakeoff::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("work_item_example", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}


extern "C" __EXPORT int marine_takeoff_main(int argc, char *argv[])
{
    return MarineTakeoff::main(argc, argv);
}

