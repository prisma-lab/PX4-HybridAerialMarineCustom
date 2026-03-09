#pragma once
// CUSTOM MODE
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <frequency_estimator/FrequencyEstimator.hpp> // Includi la libreria
#include <high_frequency_topic_selector/TopicSelector.hpp>



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

	// Istanze per stimare la frequenza dei due topic
	FrequencyEstimator _att_freq_est;
	FrequencyEstimator _imu_freq_est;

	float _freq_att{0.0f};
	float _freq_imu{0.0f};
	uint64_t next_sample{0};
	static constexpr int FFT_SIZE = 256;  // dimensione della FFT
	float accel_buffer[FFT_SIZE][3];        // buffer per roll / attitude
	float q_buffer[FFT_SIZE][4];         // buffer per imu vibration / ZOH
	int idx{0};
	bool _is_logging{false};  // Questo è l'interruttore: false = spento, true = acceso
   	int _file_counter{0};     // Serve per dare nomi diversi ai file: log_0, log_1, ecc.                        // indice corrente nel buffer

	int  _best_index{-1};        // Qui salviamo il numero del vincitore (0 o 1)
	bool _att_was_faster{true};  // Ci serve per ricordarci chi ha vinto l'ultima volta

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig
	)
};
