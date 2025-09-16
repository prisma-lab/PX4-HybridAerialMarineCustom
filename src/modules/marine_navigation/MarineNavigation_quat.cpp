// CUSTOM MODE
#include "MarineNavigation_quat.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

MarineNavigation::MarineNavigation() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

MarineNavigation::~MarineNavigation()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MarineNavigation::init()
{
	ScheduleOnInterval(100_ms); // 10Hz
	return true;
}

void MarineNavigation::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Update parameters if needed
	if (_parameter_update_sub.updated()) {
		parameter_update_s p{};
		_parameter_update_sub.copy(&p);
		updateParams();
	}

	float dt{0.01f}; // Default time step for integration
	float now = hrt_absolute_time() * 1e-6; 
	if ((double)last_timestamp > 1e-6) {
		dt = now - last_timestamp;
	}
	last_timestamp = now;

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);
	}

	if (_manual_control_sub.update(&rc_input) && vehicle_control_mode.flag_control_prisma_marine_manual_enabled) {
		
		Vector3f rpy;
		// Receiving feedback and normalizimg between -1 and 1
		if (_vehicle_attitude_sub.updated()) {
   			_vehicle_attitude_sub.copy(&vehicle_attitude);
   			rpy = getRPY(Quatf(vehicle_attitude.q));

			if(module_initialization) {
				yaw_cont = yaw_cont + wrap(rpy(2) - yaw_fb_prev, M_PI); // Continuous yaw angle
				yaw_fb_prev = rpy(2);
				quat_fb = Quatf(cos(yaw_cont / 2), 0, 0, -sin(yaw_cont / 2)); // Quaternion feedback from yaw angle
			}

			else if(!module_initialization) {
				yaw_cont = rpy(2);
				yaw_fb_prev = rpy(2);
				quat_fb = Quatf(cos(rpy(2) / 2), 0, 0, -sin(rpy(2) / 2)); // Quaternion feedback from yaw angle
				quat_d = quat_fb; // Initialize desired quaternion with the first feedback
				module_initialization = true; // Set flag to true after initialization
			}
		}

		if (_vehicle_angular_velocity_sub.updated()) {
   			_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
	  		yaw_rate_fb = vehicle_angular_velocity.xyz[2]; 
 		}
		PX4_INFO("Y fb: %.2f, YR fb : %.2f", (double)rpy(2), (double)yaw_rate_fb);
		PX4_INFO("Omega z input: %.2f", (double)computeOmegaInput(rc_input.roll));
		PX4_INFO("Omega z error: %.2f", (double)(computeOmegaInput(rc_input.roll) - yaw_rate_fb));
		// Update quaternion desired based on angular velocity input
		updateQDesired(dt, computeOmegaInput(rc_input.roll)); 
		// Compute quaternion error
		quat_error = quat_d * quat_fb.inversed();
		quat_error.normalize();
		if (fabs(quat_error(3)) < 0.009f){
			quat_error(3) = 0.0f;
		}
		// Compute omega desired and deal with unwinding
		Vector3f omega_d = Vector3f(quat_error(1), quat_error(2), quat_error(3)) * K_q * std::copysign(1.0f, quat_error(0));
		PX4_INFO("Omega d: %.2f, %.2f, %.2f", (double)omega_d(0), (double)omega_d(1), (double)omega_d(2));
		float yaw_control_input = K_r * (-omega_d(2) + yaw_rate_fb);

		// Print throttle and yaw control inputs
		PX4_INFO("Throttle: %.2f, Roll input : %.2f, Torque Input: %.2f", double(rc_input.throttle), (double)rc_input.roll, double(yaw_control_input));
		PX4_INFO("quat_d [0]: %.2f, quat_fb [0]: %.2f", (double)quat_d(0), (double)quat_fb(0));
		PX4_INFO("quat_d [1]: %.2f, quat_fb [1]: %.2f", (double)quat_d(1), (double)quat_fb(1));
		PX4_INFO("quat_d [2]: %.2f, quat_fb [2]: %.2f", (double)quat_d(2), (double)quat_fb(2));
		PX4_INFO("quat_d [3]: %.2f, quat_fb [3]: %.2f", (double)quat_d(3), (double)quat_fb(3));
		PX4_INFO("Quat error [0]: %.2f, Quat error [3]: %.2f, YR error : %.2f", (double)quat_error(0), (double)quat_error(3), (double)(omega_d(2) - yaw_rate_fb));
		PX4_INFO("angulare error norm: %.2f", (double)(2.0f * acosf(fabsf(quat_error(0)))));
		control_input = getControlInput(rc_input.throttle, yaw_control_input);
		PX4_INFO("Control Input Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));

		// Publish on actiuator_servo topic even if not armed
		actuator_servos_s actuator_servos{};
		actuator_servos.control[0] = control_input(0); // Left propeller control input
		actuator_servos.control[1] = control_input(1); // Right propeller
		_actuator_servos_pub.publish(actuator_servos);	
	}
	// If NOT in manual control mode, stop the servos
	else if(!vehicle_control_mode.flag_control_prisma_marine_manual_enabled) {
		actuator_servos_s actuator_servos{};
		actuator_servos.control[0] = 0; 
		actuator_servos.control[1] = 0; 
		_actuator_servos_pub.publish(actuator_servos);
		// Publish on orb_test topic	

		// Reset variables
		if (module_initialization) {
			module_initialization = false; // Reset initialization flag
		}
	}

	perf_end(_loop_perf);
}

Vector3f MarineNavigation::getRPY(const Quatf &q)
{
	Eulerf euler(q);
	return Vector3f(euler(0), euler(1), euler(2));
}

void MarineNavigation::updateQDesired(const float &d_t, const float &omega_z)
{
	float leak = leak_factor; // Leak factor for the state variable
	Quatf quat_d_n1 = Quatf(quat_d(0)*cos(omega_z * d_t / 2) + quat_d(3) * sin(omega_z * d_t / 2) - d_t * leak * (quat_d(0) - quat_fb(0)), 
	0, 
	0, 
	- quat_d(0)*sin(omega_z * d_t / 2) + quat_d(3) * cos(omega_z * d_t / 2) - d_t * leak * (quat_d(3) - quat_fb(3))); // Update desired quaternion considering leakage
	quat_d_n1.normalize();
	quat_d = quat_d_n1;
}

Vector2f MarineNavigation::getControlInput(const float &throttle_input, const float &yaw_speed_input)
{
	Vector2f computed_input;
	// Calculate the control input for each thruster based on the throttle and yaw speed inputs
	computed_input(0) = (0.3f * 2 * throttle_input + yaw_speed_input)/ 0.6f;
	computed_input(1) = (0.3f * 2 * throttle_input - yaw_speed_input)/ 0.6f;

	if (computed_input(0) > 1.0f) {
		computed_input(0) = 1.0f; // Clamp left thruster input to 1
	} else if (computed_input(0) < -1.0f) {
		computed_input(0) = -1.0f; // Clamp left thruster input to -1
	}
	if (computed_input(1) > 1.0f) {
		computed_input(1) = 1.0f; // Clamp right thruster input to 1
	} else if (computed_input(1) < -1.0f) {
		computed_input(1) = -1.0f; // Clamp right thruster input to -1
	}

	return computed_input;
}

float MarineNavigation::computeOmegaInput(const float &omega_input)
{
	float omega_filtered;
	if (omega_input <= 0.01f && omega_input >= -0.01f) {
		omega_filtered = 0.0f;	
	}
	else {
		omega_filtered = omega_input; 
	}

	omega_filtered = 0.9f * omega_filtered * MAX_YAW_SPEED;

	return omega_filtered;
}

float MarineNavigation::wrap(const float &angle, const float &wrap_number)
{
	float wrapped_angle = fmodf(angle + wrap_number, 2.0f * wrap_number);
	if (wrapped_angle < 0) {
		wrapped_angle += 2.0f * wrap_number; // Ensure the angle is positive
	}
	return wrapped_angle - wrap_number; // Return the angle wrapped to [-wrap_number, wrap_number]
}

int MarineNavigation::task_spawn(int argc, char *argv[])
{
	MarineNavigation *instance = new MarineNavigation();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MarineNavigation::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MarineNavigation::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MarineNavigation::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Reads throttle and roll stick inputs from RC (manual_control_setpoint).
No arming required.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("marine_navigation", "custom");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int marine_navigation_main(int argc, char *argv[])
{
	return MarineNavigation::main(argc, argv);
}