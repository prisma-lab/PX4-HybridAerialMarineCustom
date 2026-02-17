#pragma once
// CUSTOM MODE
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <lib/geo/geo.h>
#include <lib/events/events.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>

#include <matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/mavlink_log.h> 
#include <uORB/topics/marine_navigation.h>
#include <uORB/topics/position_setpoint_triplet.h>

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
	Vector2f getControlInput(const float &force_input, const float &yaw_speed_input); 
	float computeOmegaInput(const float &omega_input);  
	float wrap(const float &angle, const float &wrap_number = M_PI); // Default wrap number is M_PI for [-M_PI, M_PI] wrapping

	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_armed_s> _armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<marine_navigation_s> _marine_navigation_pub{ORB_ID(marine_navigation)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	orb_advert_t _mavlink_log_pub{nullptr};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
 	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	manual_control_setpoint_s rc_input{};
	vehicle_control_mode_s vehicle_control_mode{};
	vehicle_angular_velocity_s vehicle_angular_velocity{};
	vehicle_attitude_s vehicle_attitude{};
	vehicle_local_position_s vehicle_local_position{};
	marine_navigation_s marine_navigation{}; // Marine navigation topic instance
	position_setpoint_triplet_s	_pos_sp_triplet{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,
		(ParamFloat<px4::params::MARINE_PROP_C>) _param_marine_prop_c,
		(ParamFloat<px4::params::MARINE_KQ>) _param_marine_kq,
		(ParamFloat<px4::params::MARINE_KR>) _param_marine_kr,
		(ParamFloat<px4::params::MARINE_KT>) _param_marine_kt,
		(ParamFloat<px4::params::MARINE_KT_FF>) _param_marine_kt_ff,
		(ParamFloat<px4::params::MARINE_LEAK>) _param_marine_leak,
		(ParamFloat<px4::params::MARINE_DRAG>) _param_marine_drag,
		(ParamFloat<px4::params::MARINE_MAX_SP>) _param_marine_max_speed,
		(ParamFloat<px4::params::MARINE_MAX_YAW>) _param_marine_max_yaw,
		(ParamFloat<px4::params::MARINE_L_X>) _param_marine_l_x,
		(ParamFloat<px4::params::MARINE_L_Y>) _param_marine_l_y,
		(ParamFloat<px4::params::MARINE_R_X>) _param_marine_r_x,
		(ParamFloat<px4::params::MARINE_R_Y>) _param_marine_r_y,	
		// Auto mission params
		(ParamFloat<px4::params::AUTO_LPF>) _param_auto_lpf,
		(ParamFloat<px4::params::AUTO_K_PX>) _param_auto_k_px,
		(ParamFloat<px4::params::AUTO_K_PY>) _param_auto_k_py,
		(ParamFloat<px4::params::AUTO_K_VX>) _param_auto_k_vx,
		(ParamFloat<px4::params::AUTO_K_VY>) _param_auto_k_vy,
		(ParamFloat<px4::params::AUTO_K_IX>) _param_auto_k_Ix,
		(ParamFloat<px4::params::AUTO_K_IY>) _param_auto_k_Iy,
		(ParamFloat<px4::params::AUTO_K_IL>) _param_auto_k_il,
		(ParamFloat<px4::params::AUTO_I_MAX>) _param_auto_i_max,
		(ParamFloat<px4::params::AUTO_K_C>) _param_auto_k_c,
		(ParamFloat<px4::params::AUTO_K_CL>) _param_auto_k_cl,
		(ParamFloat<px4::params::AUTO_VC_MAX>) _param_auto_vc_max,
		(ParamFloat<px4::params::AUTO_VMIN_ADAPT>) _param_auto_vmin_adapt,
		(ParamFloat<px4::params::AUTO_V_CRUISE>) _param_v_cruise_auto,
		(ParamFloat<px4::params::AUTO_D_SLOW>) _param_d_slow_auto
	)

	// Parameter variables
	float Prop_C{_param_marine_prop_c.get()};
	float K_q{_param_marine_kq.get()};
	float K_r{_param_marine_kr.get()};
	float K_t{_param_marine_kt.get()};
	float K_t_ff{_param_marine_kt_ff.get()};
	float leak{_param_marine_leak.get()};
	float drag_coeff{_param_marine_drag.get()};
	float max_propeller_speed{_param_marine_max_speed.get()};
	float max_yawspeed{_param_marine_max_yaw.get()};
	float left_th_x{_param_marine_l_x.get()};
	float left_th_y{_param_marine_l_y.get()};
	float right_th_x{_param_marine_r_x.get()};
	float right_th_y{_param_marine_r_y.get()};
	// Auto mission params
	float l_pf{_param_auto_lpf.get()};
	float k_px{_param_auto_k_px.get()};
	float k_py{_param_auto_k_py.get()};
	float k_vx{_param_auto_k_vx.get()};
	float k_vy{_param_auto_k_vy.get()};
	float k_Ix{_param_auto_k_Ix.get()};
	float k_Iy{_param_auto_k_Iy.get()};
	float k_il{_param_auto_k_il.get()};
	float I_max{_param_auto_i_max.get()};
	float k_c{_param_auto_k_c.get()};
	float k_cl{_param_auto_k_cl.get()};
	float vc_max{_param_auto_vc_max.get()};
	float vmin_adapt{_param_auto_vmin_adapt.get()};
	float v_cruise_auto{_param_v_cruise_auto.get()};
	float d_slow_auto{_param_d_slow_auto.get()};

	// Control variables
	float v_x;
	float yaw_cont;
	float yaw_fb_prev;
	Quatf quat_fb;
	Quatf quat_d; // Desired quaternion
	Quatf quat_error;
	float yaw_rate_fb;
	float yaw_rate_error;
	bool module_initialization{false}; // Flag to check if integral is initialized
	float last_timestamp{0};
	Vector2f control_input{0, 0}; // Control inputs for the propellers

	// Autonomous navigation params
	float s_pf = 1.0f;  // arclength [m]
	float xi1_I = 0.0f;  
	float xi2_I = 0.0f;  
	float vc_hat_x = 0.0f; 
	float vc_hat_y = 0.0f; 

	MapProjection _global_local_proj_ref{};

	bool in_marine_mode{false};
};