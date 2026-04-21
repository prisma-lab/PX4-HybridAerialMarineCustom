
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
     if (_parameter_update_sub.updated()) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams(); // update module parameters (in DEFINE_PARAMETERS)
    }


    // Example
    //  update vehicle_status to check arming state
    if (_vehicle_status_sub.updated()) {
        vehicle_status_s vehicle_status;

        if (_vehicle_status_sub.copy(&vehicle_status)) {

            const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
            _armed = armed;
        }
    }




    _vehicle_control_mode_sub.update(&_vehicle_control_mode);

    //if(_vehicle_control_mode.flag_control_marine_takeoff_enabled){
    //  PX4_INFO("Marine Takeoff mode enabled");
    //
     if (!_armed) {
        // Arm the vehicle
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0f; // 1 = arm
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = false;

        _vehicle_command_pub.publish(cmd);
        PX4_INFO("Sent ARM command from MarineTakeoff module");
    }




    // If armed but not yet taken off
    //if (_armed && !_takeoff_sent) {
    //
    //  vehicle_command_s mode_cmd{};
    //  mode_cmd.timestamp = hrt_absolute_time();
    //  mode_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;

    //  mode_cmd.param1 = 1; // custom mode enabled
    //  mode_cmd.param2 = 4;
    //  mode_cmd.param3 = 2; // AUTO_TAKEOFF  Mode  for takeoff

    //  mode_cmd.target_system = 1;
    //  mode_cmd.target_component = 1;
    //  mode_cmd.source_system = 1;
    //  mode_cmd.source_component = 1;
    //  mode_cmd.from_external = false;

    //  _vehicle_command_pub.publish(mode_cmd);

    //  PX4_INFO("Sent AUTO_TAKEOFF mode command");

    //  _takeoff_sent = true;

    //}
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

            const  float * sources[] = { &acc_z_pure, &_last_wave.wave_amplitude };
            float* targets[] = { _buf_az, _raw_wave_buf };

        // Riempie i buffer alla posizione _fft_idx
            _zoh_processor.bufferDataMulti(sources, targets, 2, _fft_idx, FFT_SIZE);


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
    //}
}

void MarineTakeoff::process_fft_and_snapshot()
{


    for (size_t i = 0; i < FFT_SIZE; i++) {
        size_t circ_idx = (_fft_idx + i) % FFT_SIZE;
        work_buf[i] = _buf_az[circ_idx];
        _final_wave_gt_send[i] = _raw_wave_buf[circ_idx];
    }

    kiss_fftr(_fft_cfg, work_buf, _fft_out);


      // --- TRUCCO DELLA COPIA ---
    // Salviamo l'accelerazione pura in una copia prima di modificarla
     static kiss_fft_cpx fft_copy[FFT_SIZE / 2 + 1];
     memcpy(fft_copy, _fft_out, sizeof(_fft_out));

    _fft_out[0].r = 0.0f; _fft_out[0].i = 0.0f;
    const float Fs=250.0f;
    const float df = Fs/ (float)FFT_SIZE;
    const float dt = 1.0f / Fs;

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

      // --- STEP B: PREDIZIONE (Il Futuro) ---
    // Usiamo la COPIA (fft_copy) che ha ancora l'accelerazione pura!
    for (int i = 0; i < (int)FFT_SIZE; i++) {
        float tp = (float)(FFT_SIZE + i) * dt;
        float h_p = 0.0f;

        // Limite a 200 (~2Hz) per non sovraccaricare la CPU del drone
        for (int k = 1; k < 200; k++) {
            float freq = k * df;
            if (freq >= 0.05f) {
                // Leggiamo dalla COPIA
                float mag_a = sqrtf(fft_copy[k].r * fft_copy[k].r + fft_copy[k].i * fft_copy[k].i) * 2.0f / FFT_SIZE;
                float phase_a = atan2f(fft_copy[k].i, fft_copy[k].r);
                float omega = 2.0f * M_PI_F * freq;

                h_p += (mag_a / (omega * omega)) * cosf(omega * tp + phase_a);
            }
        }
        _prediction_send[i] = h_p;
    }

    _is_sending_results = true;
    _current_chunk_idx = 0;
    _cycle_count++;
    _total_samples = 0;

    // --- AGGIUNGO  QUESTI PRINT QUI ---
    float last_past = _displacement_send[FFT_SIZE - 1];
    float first_future = _prediction_send[0];

    PX4_INFO("Confronto Giunzione (NED): Passato[Fine]: %.3f | Futuro[Inizio]: %.3f",
              (double)last_past, (double)first_future);

    // Se i valori hanno segni opposti, questo print ti avvisa:
    if ((last_past > 0 && first_future < 0) || (last_past < 0 && first_future > 0)) {
        PX4_WARN("ATTENZIONE: L'onda è ribaltata! Controlla i segni.");}

    vehicle_command_s save_msg{};
    save_msg.command = 999;
    _vehicle_command_pub.publish(save_msg);
    PX4_INFO("FFT completata. Invio dati...");
}

void MarineTakeoff::SendChunks()
{
    // 1. CONTROLLO TEMPORALE (PACING)
    // Impedisce l'invio di più di un chunk ogni 12 millisecondi
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



