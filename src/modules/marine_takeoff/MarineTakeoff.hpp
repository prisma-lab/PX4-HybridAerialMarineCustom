#pragma once
// CUSTOM MODE
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include<zoh_utils/ZohUtils.hpp>
#include<kissFFT/kiss_fftr.h>



#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/marine_sync.h>







using namespace time_literals;

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






	// Publications (example only)
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_armed_s> _armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<marine_sync_s> _marine_sync_pub{ORB_ID(marine_sync)};




	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	//RB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};






	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	vehicle_control_mode_s _vehicle_control_mode{};
	bool _armed{false};
	bool _takeoff_sent {false};
	float _home_alt_msl{0.0f};


	vehicle_attitude_s   _last_attitude{};
        vehicle_imu_status_s _last_imu_status{};

         zoh_utils::ZOHProcessor _zoh_processor;

        uint64_t next_sample{0};


        bool _is_logging{false};
        int _file_counter{0};

        marine_sync_s _sync_msg{};


	static constexpr size_t FFT_SIZE = 128;

	// 4 (Attitude)
	float _buf_q0[FFT_SIZE];
	float _buf_q1[FFT_SIZE];
	float _buf_q2[FFT_SIZE];
	float _buf_q3[FFT_SIZE];

	// 3  Acceleration (Movement)
	float _buf_ax[FFT_SIZE];
	float _buf_ay[FFT_SIZE];
	float _buf_az[FFT_SIZE];

	int _fft_idx{0}; // Shared counter for all

	// FFT tools
	kiss_fftr_cfg _fft_cfg{nullptr};
	kiss_fft_cpx _fft_out[FFT_SIZE / 2 + 1];

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig
	)
};
