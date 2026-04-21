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
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_imu_status.h>
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
    void process_fft_and_snapshot();
    void SendChunks();

    // --- Costanti e Buffer ---
    static constexpr size_t FFT_SIZE = 2500;
    float _buf_az[FFT_SIZE]{};
    float _raw_wave_buf[FFT_SIZE]{};
    float _displacement_output[FFT_SIZE]{};

    // Buffer snapshot per invio DDS
    float _final_wave_gt_send[FFT_SIZE]{};
    float _displacement_send[FFT_SIZE]{};
    float work_buf[FFT_SIZE]{};
    float _prediction_send[FFT_SIZE]{};




    // Buffer temporaneo per la FFT (nella classe per evitare stack overflow)
    float _work_buf[FFT_SIZE]{};

    // --- Stato ---
    int   _fft_idx{0};
    int   _total_samples{0};
    int   _cycle_count{0};
    const int _max_cycles = 3;
    bool  _is_sending_results{false};
    int   _current_chunk_idx{0};
    bool  _armed{false};

    // --- KissFFT ---
    kiss_fftr_cfg _fft_cfg{nullptr};      // <--- DICHIARATI QUI
    kiss_fftr_cfg _fft_inv_cfg{nullptr};  // <--- DICHIARATI QUI
    kiss_fft_cpx  _fft_out[FFT_SIZE / 2 + 1]{};

    // --- uORB ---
    uORB::Publication<marine_sync_s> _marine_sync_pub{ORB_ID(marine_sync)};
    uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
    uORB::Publication<marine_wave_result_s> _marine_wave_result_pub{ORB_ID(marine_wave_result)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
    uORB::Subscription _marine_wave_sub{ORB_ID(marine_wave)};
    uORB::Publication<marine_wave_prediction_s> _marine_prediction_pub{ORB_ID(marine_wave_prediction)};


    ;

    marine_wave_prediction_s _pred_msg{};
    marine_wave_s        _last_wave{};
    vehicle_attitude_s   _last_attitude{};
    vehicle_imu_status_s _last_imu_status{};
    marine_wave_result_s _log_msg{};
    marine_sync_s        _sync_msg{};
    zoh_utils::ZOHProcessor _zoh_processor;

    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
        (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig
    )
};
