#pragma once
// CUSTOM MODE
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>

#include <matrix/math.hpp>

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
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace time_literals;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::Vector2f;
using matrix::SquareMatrix;

class MarineNavigation : public ModuleBase<MarineNavigation>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MarineNavigation();
	~MarineNavigation() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	Vector3f getRPY(const Quatf &q); 
	void updateQDesired(const float &d_t, const float &omega_z); 
	Vector2f getControlInput(const float &throttle_input, const float &yaw_speed_input); 
	float computeOmegaInput(const float &omega_input);  
	float wrap(const float &angle, const float &wrap_number = M_PI); // Default wrap number is M_PI for [-M_PI, M_PI] wrapping

	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_armed_s> _armed_pub{ORB_ID(actuator_armed)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
 	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	manual_control_setpoint_s rc_input{};
	vehicle_control_mode_s vehicle_control_mode{};
	vehicle_angular_velocity_s vehicle_angular_velocity{};
	vehicle_attitude_s vehicle_attitude{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,
		(ParamFloat<px4::params::MARINE_PROP_C>) _param_marine_prop_c,
		(ParamFloat<px4::params::MARINE_KQ>) _param_marine_kq,
		(ParamFloat<px4::params::MARINE_KR>) _param_marine_kr,
		(ParamFloat<px4::params::MARINE_LEAK>) _param_marine_leak,
		(ParamFloat<px4::params::MARINE_MAX_TH>) _param_marine_max_th,
		(ParamFloat<px4::params::MARINE_MAX_YAW>) _param_marine_max_yaw,
		(ParamFloat<px4::params::MARINE_L_X>) _param_marine_l_x,
		(ParamFloat<px4::params::MARINE_L_Y>) _param_marine_l_y,
		(ParamFloat<px4::params::MARINE_R_X>) _param_marine_r_x,
		(ParamFloat<px4::params::MARINE_R_Y>) _param_marine_r_y	
	)

	// Parameter variables
	float Prop_C{_param_marine_prop_c.get()};
	float K_q{_param_marine_kq.get()};
	float K_r{_param_marine_kr.get()};
	float leak{_param_marine_leak.get()};
	float max_propeller_th{_param_marine_max_th.get()};
	float max_yawspeed{_param_marine_max_yaw.get()};
	float left_th_x{_param_marine_l_x.get()};
	float left_th_y{_param_marine_l_y.get()};
	float right_th_x{_param_marine_r_x.get()};
	float right_th_y{_param_marine_r_y.get()};

	// Control variables
	float yaw_cont;
	float yaw_fb_prev;
	Quatf quat_fb;
	Quatf quat_d; // Desired uaternion
	Quatf quat_error;
	float yaw_rate_fb;
	float yaw_rate_error;
	bool module_initialization{false}; // Flag to check if integral is initialized
	float last_timestamp{0};
	Vector2f control_input{0, 0}; // Control inputs for the propellers
};