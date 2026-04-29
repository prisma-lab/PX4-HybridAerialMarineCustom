#pragma once

#include <algorithm>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <zoh_utils/ZohUtils.hpp>
#include <kissFFT/kiss_fftr.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/marine_sync.h>
#include <uORB/topics/marine_wave.h>
#include <uORB/topics/marine_wave_result.h>
#include <uORB/topics/marine_wave_prediction.h>
#include <lib/matrix/matrix/math.hpp>

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector3f;

class MarineTakeoff : public ModuleBase<MarineTakeoff>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    MarineTakeoff();
    ~MarineTakeoff() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    bool init();
    int print_status() override;

private:
    void Run() override;

    /**
     * @brief Elabora i dati raccolti: calcola Heave passato e predice quello futuro.
     */
    void process_fft_and_snapshot();

    /**
     * @brief Invia i risultati a blocchi (chunks) via uORB/DDS per evitare saturazione.
     */
    void SendChunks();

    /**
     * @brief Esegue FFT, integrazione doppia e IFFT.
     * @param input_time_buf Segnale di accelerazione in ingresso.
     * @param output_time_buf Segnale di spostamento (Heave) in uscita.
     * @param apply_integration Se true, applica il filtro -1/w^2.
     */
    void double_int(float* input_time_buf, float* output_time_buf, bool apply_integration, bool is_real_data);

    /**
     * @brief Predice l'accelerazione futura basandosi sui coefficienti FFT salvati.
     */
    void predict_wave(float* prediction_buf, int prediction_samples);

    // --- Costanti e Buffer di Memoria ---
    static constexpr size_t FFT_SIZE = 5000;

    // Buffer circolari per la raccolta dati
    float _buf_az[FFT_SIZE]{};           // Accelerazione lineare verticale
    float _raw_wave_buf[FFT_SIZE]{};     // Ground Truth (da simulatore)

    // Buffer di lavoro (snapshot) per elaborazione
    float work_buf[FFT_SIZE]{};          // Buffer temporaneo multiuso
    float _displacement_send[FFT_SIZE]{}; // Heave passato calcolato (20s)
    float _prediction_send[FFT_SIZE]{};   // Heave futuro predetto (20s)
    float _final_wave_gt_send[FFT_SIZE]{}; // Ground Truth salvata
    float _prediction_old[FFT_SIZE]{};

    // --- Stato del Modulo ---
    int   _fft_idx{0};                   // Indice corrente nel buffer circolare
    int   _total_samples{0};             // Campioni raccolti nel ciclo attuale
    int   _cycle_count{0};               // Numero di FFT completate
    const int _max_cycles = 3;          // Numero massimo di cicli di test
    bool  _is_sending_results{false};    // Flag per lo streaming dei dati
    int   _current_chunk_idx{0};         // Indice del chunk uORB in invio
    bool  _armed{false};                 // Stato armamento drone
    float windowed_buf[FFT_SIZE];
    float _windowed_buf_member[FFT_SIZE];


    // --- KissFFT ---
    kiss_fftr_cfg _fft_cfg{nullptr};      // Configurazione FFT reale
    kiss_fftr_cfg _fft_inv_cfg{nullptr};  // Configurazione IFFT reale
    kiss_fft_cpx  _fft_out[FFT_SIZE / 2 + 1]{};       // Output FFT corrente
    kiss_fft_cpx  _fft_accel_copy[FFT_SIZE / 2 + 1]{}; // Backup coefficienti accelerazione pura
    kiss_fft_cpx _fft_pure[FFT_SIZE / 2 + 1] {};
    // --- uORB Subscriptions & Publications ---
    uORB::Publication<marine_sync_s>            _marine_sync_pub{ORB_ID(marine_sync)};
    uORB::Publication<vehicle_command_s>        _vehicle_command_pub{ORB_ID(vehicle_command)};
    uORB::Publication<marine_wave_result_s>     _marine_wave_result_pub{ORB_ID(marine_wave_result)};
    uORB::Publication<marine_wave_prediction_s> _marine_prediction_pub{ORB_ID(marine_wave_prediction)};

    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
    uORB::Subscription _marine_wave_sub{ORB_ID(marine_wave)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // Messaggi uORB
    vehicle_control_mode_s     _vehicle_control_mode{};
    marine_wave_s              _last_wave{};
    vehicle_attitude_s         _last_attitude{};
    vehicle_imu_status_s       _last_imu_status{};
    marine_wave_result_s       _log_msg{};
    marine_wave_prediction_s   _pred_msg{};

    // --- Utility ---
    zoh_utils::ZOHProcessor    _zoh_processor;
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
        (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig
    )
};
