#pragma once
// CUSTOM MODE
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

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

#define MAX_YAW_SPEED 0.24f // Maximum yaw speed in rad/s
#define MAX_PROP_THRUST 1500.0f // Maximum propeller thrust (normalized, 0 to 1)
#define K_p 2.5f // Proportional gain for yaw control
#define K_i 1.8f // Integral gain for yaw control
#define LEFT_TH_X -0.53 // Left thruster X position
#define LEFT_TH_Y 0.3 // Left thruster Y position
#define RIGHT_TH_X -0.53 // Right thruster X position
#define RIGHT_TH_Y -0.3 // Right thruster Y position
#define DRONE_MASS 16.0f // Mass of the drone in kg

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
	void forward_euler_integration(const float &d_t, const float &u_n);
	float compute_yaw_error(const float &y_d, const float &y_fb);
	Vector2f getControlInput(const float &throttle_input, const float &yaw_speed_input); 
	float filterYawInput(const float &yaw_input);
	float wrap(const float &angle, const float &wrap_number = 1.0f); // Default wrap number is 1.0 for [-1, 1] wrapping

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
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig
	)

	// Control variables
	float max_propeller_trust{MAX_PROP_THRUST};
	float max_yawspeed{MAX_YAW_SPEED};
	float yaw_fb;
	float yaw_rate_fb;
	float yaw_input_integral;
	bool module_initialization{false}; // Flag to check if integral is initialized
	float last_timestamp{0};
	Vector2f control_input{0, 0}; // Control inputs for the propellers
};