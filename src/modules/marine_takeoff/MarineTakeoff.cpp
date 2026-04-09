#include "MarineTakeoff.hpp"
#include  <fstream>
MarineTakeoff::MarineTakeoff() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

MarineTakeoff::~MarineTakeoff()
{
    if (_fft_cfg) {
                free(_fft_cfg);
        }
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool MarineTakeoff::init()
{

        _fft_cfg = kiss_fftr_alloc(FFT_SIZE, 0, nullptr, nullptr);
    _fft_inv_cfg=kiss_fftr_alloc(FFT_SIZE, 1, nullptr, nullptr);
        if (_fft_cfg == nullptr) {
                PX4_ERR("FFT allocation failed!");
                return false;
        }
    // alternatively, Run on fixed interval
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

    // Check if parameters have changed
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

            if (armed && !_armed) {
                PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

            } else if (!armed && _armed) {
                PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
            }

            _armed = armed;
        }
    }


    // Example
    //  grab latest accelerometer data
    if (_sensor_accel_sub.updated()) {
        sensor_accel_s accel;

        if (_sensor_accel_sub.copy(&accel)) {
            // DO WORK

            // access parameter value (SYS_AUTOSTART)
            if (_param_sys_autostart.get() == 1234) {
                // do something if SYS_AUTOSTART is 1234
            }
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



        // ======================================================
      //  UPDATE  VEHICLE_ATTITUDE AND VEHICLE_IMU_STATUS
    // ======================================================


        const bool attitude_updated = _vehicle_attitude_sub.updated();
    const bool imu_status_updated = _vehicle_imu_status_sub.updated();

    if (attitude_updated) {
        _vehicle_attitude_sub.copy(&_last_attitude);
    }

    if (imu_status_updated) {
        _vehicle_imu_status_sub.copy(&_last_imu_status);
    }

       // ======================================================
    // ZOH + Sampling at fastest topic update
    // ======================================================
        zoh_utils::ZOHProcessor::Input streams[] = {
        {_last_attitude.q, _last_attitude.timestamp, sizeof(_last_attitude.q), attitude_updated},
        {_last_imu_status.mean_accel, _last_imu_status.timestamp, sizeof(_last_imu_status.mean_accel), imu_status_updated}
    };

    auto result = _zoh_processor.process(streams, 2);

    if (result.updated) {
        // Sync data
        zoh_utils::ZOHProcessor::synchronize(_last_imu_status.mean_accel, _sync_msg.accel, sizeof(_sync_msg.accel));
        _marine_sync_pub.publish(_sync_msg);


        // 2. create quaternion use synchronized data
    Quatf q(_last_attitude.q);

    // 3. create accelleration vector
    Vector3f acc_body(_sync_msg.accel[0], _sync_msg.accel[1], _sync_msg.accel[2]);

    // 4. Rotation from Body Frame to   World Frame

    Vector3f acc_world = q.rotateVector(acc_body);

    // 5. Gravity Compensation
    // PX4 follows the NED (North-East-Down) standard.
    // When at rest, the world-frame Z-acceleration (Down) is approximately -9.80665 m/s².
    // We add +9.80665f to center the signal at zero.
    // Note: In this centered signal, POSITIVE values indicate downward acceleration,
    // and NEGATIVE values indicate upward acceleration.
    float acc_z_pure = acc_world(2) + 9.80665f;




// --- VARIABILI DI STATO ---
// Devono essere static per non essere resettate ad ogni Run()
static bool finished = false;
static int total_samples = 0;
static float raw_acc_z_buffer[FFT_SIZE];
static float final_raw_acc[FFT_SIZE];

if (!finished) {
    if (total_samples == 0) {
        vehicle_command_s start_msg{};
        start_msg.timestamp = hrt_absolute_time();
        start_msg.command = 888; // ID Sincronizzazione: INIZIO RACCOLTA
        start_msg.param1 = 1.0f;
        _vehicle_command_pub.publish(start_msg);
        PX4_INFO("ROS2: Inizia a registrare ORA!");
    }
    // 1. Salva il dato grezzo prima dell'incremento dell'indice
    raw_acc_z_buffer[_fft_idx] = acc_z_pure;

    // 2. Preparazione liste per ZOH (Dichiarate UNA SOLA VOLTA)
    const float* src_list[] = { &acc_z_pure };
    float* dst_list[] = { _buf_az };

    // Inserimento nel buffer (incrementa _fft_idx internamente)
    _zoh_processor.bufferDataMulti(src_list, dst_list, 1, _fft_idx, FFT_SIZE);

    total_samples++;

    // 3. Quando arriviamo a 10 secondi (2500 campioni)
    if (total_samples >= 5000) {


        finished = true;
        // Inviamo un comando che ROS2 può leggere

        vehicle_command_s save_msg{};
        save_msg.timestamp = hrt_absolute_time();
        save_msg.command = 999; // ID Sincronizzazione: FINE E SALVA
        _vehicle_command_pub.publish(save_msg);


        PX4_INFO("10 secondi raggiunti. Inizio elaborazione...");

        static float work_buf[FFT_SIZE];
        float sum = 0.0f;

        // --- STEP: SROTOLAMENTO ---
        for (size_t i = 0; i < FFT_SIZE; i++) {
            size_t circular_idx = (_fft_idx + i) % FFT_SIZE;
            work_buf[i] = _buf_az[circular_idx];
            final_raw_acc[i] = raw_acc_z_buffer[circular_idx];
            sum += work_buf[i];
        }


        // --- STEP: FFT ---
        kiss_fftr(_fft_cfg, work_buf, _fft_out);

    ///   fino a questo punto non ho alcun errore da qui in poi devo ancora testare





     // 1. Reset delle componenti DC (Frequenza 0Hz)
_fft_out[0].r = 0.0f;
_fft_out[0].i = 0.0f;

int num_bins = (FFT_SIZE / 2) + 1;
bool is_peak[num_bins];
// Definizioni per il calcolo delle frequenze
const float Fs = 250.0f; // Frequenza di campionamento (es. 250Hz per 2500 campioni in 10s)
const float df = Fs / (float)FFT_SIZE;
const float two_pi = 2.0f * M_PI_F;

for (int k = 1; k < num_bins; k++) {
    // Calcolo del modulo per la condizione is_peak (emula abs(X) di MATLAB)
    // Se is_peak[k] è già calcolato esternamente, questa parte valida la logica

    if (is_peak[k]) {
        float freq = k * df;
        float omega = two_pi * freq;

        // La doppia integrazione: divisione per -(omega^2)
        // Usiamo un segno negativo per mantenere la coerenza fisica (inversione di fase)
        float factor = 1.0f / -(omega * omega);

        _fft_out[k].r *= factor;
        _fft_out[k].i *= factor;

    }
}

// --- STEP: IFFT E RICOSTRUZIONE TEMPORALE ---

// Eseguiamo la trasformata inversa (Complex-to-Real)
kiss_fftri(_fft_inv_cfg, _fft_out, _displacement_output);

// Normalizzazione finale
// La IFFT di KissFFT non è scalata. Dobbiamo dividere per N (FFT_SIZE).
// Se avessi usato una finestra di Hanning, qui dovresti moltiplicare anche per 2.0.
const float norm_factor = 1.0f / (float)FFT_SIZE;

for (size_t i = 0; i < FFT_SIZE; i++) {
    _displacement_output[i] *= (norm_factor * -1.0f);

}

const char *csv_path = PX4_STORAGEDIR "/fft_displacement_data.csv";
std::ofstream csv_file(csv_path);

if (csv_file.is_open()) {
    // Intestazione
    csv_file << "sample_idx,acc_raw_z,displacement_z\n";

    for (size_t i = 0; i < FFT_SIZE; i++) {
        // Scriviamo l'indice, l'accelerazione originale srotolata e lo spostamento calcolato
        csv_file << i << ","
                 << (double)final_raw_acc[i] << ","
                 << (double)_displacement_output[i] << "\n";
    }

    csv_file.close();
    PX4_INFO("Dati salvati con successo in: %s", csv_path);
} else {
    PX4_ERR("Impossibile aprire il file CSV per la scrittura!");
}

    }
}






}
                perf_end(_loop_perf);
        //}
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

