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
#include <uORB/topics/marine_window.h>
#include <px4_platform_common/log.h>

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
     * @brief Processes collected data: computes past Heave and predicts future Heave.
     */
    void process_fft_and_snapshot();

    /**
     * @brief Sends results in chunks via uORB/DDS to avoid saturation.
     */
    void SendChunks();

    /**
     * @brief Performs FFT, double integration, and IFFT.
     * @param input_time_buf Input acceleration signal.
     * @param output_time_buf Output displacement signal (Heave).
     * @param apply_integration If true, applies the -1/w^2 filter.
     */
    void double_int(float* input_time_buf, float* output_time_buf, bool apply_integration, bool is_real_data);

    /**
     * @brief Predicts future acceleration based on stored FFT coefficients.
     */
    // Aggiungi kiss_fft_cpx* fft_source
    void predict_wave(float* prediction_buf, int prediction_samples, kiss_fft_cpx* fft_source);

    void build_prediction_window();
    void check_window_validity();
    // --- Constants and Memory Buffers ---
    static constexpr size_t FFT_SIZE = 5000;

    // Circular buffers for data collection
    float _buf_az[FFT_SIZE]{};           // Vertical linear acceleration
    float _raw_wave_buf[FFT_SIZE]{};     // Ground truth (from simulator)

    // Work buffer (snapshot) for processing
    float work_buf[FFT_SIZE]{};          // Temporary multi-purpose buffer
    float _displacement_send[FFT_SIZE]{}; // Computed past Heave (20s)
    float _prediction_send[FFT_SIZE]{};   // Predicted future Heave (20s)
    float _prediction_old[FFT_SIZE]{};

    // --- Module State ---
    int   _fft_idx{0};                   // Current index in circular buffer
    int   _total_samples{0};             // Samples collected in current cycle
    int   _cycle_count{0};               // Number of completed FFT cycles
    const int _max_cycles = 3;          // Maximum number of test cycles
    bool  _is_sending_results{false};    // Data streaming flag
    int   _current_chunk_idx{0};         // uORB chunk send index
    bool  _armed{false};                 // Drone armed state
    float windowed_buf[FFT_SIZE];
    float _windowed_buf_member[FFT_SIZE];
    float roll_work[FFT_SIZE]{};
    float pitch_work[FFT_SIZE]{};
    float _final_takeoff_window[FFT_SIZE]{}; //   window construction
    float _pred_heave_velocity[FFT_SIZE]{};

    float _pred_roll_out[FFT_SIZE]{};
    float _pred_pitch_out[FFT_SIZE]{};

    // 3. Circular buffer
    float _buf_roll[FFT_SIZE]{};
    float _buf_pitch[FFT_SIZE]{};


    bool _is_window_duration_valid{false};
    bool _was_window_valid{false};
    int _valid_window_start_idx{-1};
    bool _takeoff_triggered{false};       // trigger for takeoff
    bool _takeoff_completed{false};       // Blocca la FFT dopo il decollo
    hrt_abstime _target_takeoff_time{0};   // Memorizza il millisecondo esatto
    hrt_abstime _snapshot_timestamp{0};
    hrt_abstime _current_snapshot_time{0};

    // --- KissFFT ---
    kiss_fftr_cfg _fft_cfg{nullptr};      // Real FFT configuration
    kiss_fftr_cfg _fft_inv_cfg{nullptr};  // Real IFFT configuration
    kiss_fftr_cfg _fft_cfg_att{nullptr};
    kiss_fftr_cfg _fft_inv_cfg_att{nullptr};
    kiss_fft_cpx  _fft_out[FFT_SIZE / 2 + 1]{};       // Current FFT output
    kiss_fft_cpx  _fft_accel_copy[FFT_SIZE / 2 + 1]{}; // Backup of pure acceleration coefficients
    kiss_fft_cpx _fft_pure[FFT_SIZE / 2 + 1] {};
    kiss_fft_cpx  _fft_roll[FFT_SIZE / 2 + 1]{};
    kiss_fft_cpx  _fft_pitch[FFT_SIZE / 2 + 1]{};



    // --- uORB Subscriptions & Publications ---
    uORB::Publication<marine_sync_s>            _marine_sync_pub{ORB_ID(marine_sync)};
    uORB::Publication<vehicle_command_s>        _vehicle_command_pub{ORB_ID(vehicle_command)};
    uORB::Publication<marine_wave_result_s>     _marine_wave_result_pub{ORB_ID(marine_wave_result)};
    uORB::Publication<marine_wave_prediction_s> _marine_prediction_pub{ORB_ID(marine_wave_prediction)};
     uORB::Publication<marine_window_s>     _marine_window_pub{ORB_ID(marine_window)};

    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
    uORB::Subscription _marine_wave_sub{ORB_ID(marine_wave)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription   _marine_window_sub{ORB_ID(marine_window)};

    // uORB messages
    vehicle_control_mode_s     _vehicle_control_mode{};
    marine_wave_s              _last_wave{};
    vehicle_attitude_s         _last_attitude{};
    vehicle_imu_status_s       _last_imu_status{};
    marine_wave_result_s       _log_msg{};
    marine_wave_prediction_s   _pred_msg{};
    marine_window_s            _window_msg {};

    // --- Utility ---
    zoh_utils::ZOHProcessor    _zoh_processor;
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
        (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig
    )
};
