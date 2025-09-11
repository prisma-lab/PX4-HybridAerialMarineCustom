// CUSTOM MODE
#include "MarineNavigation_PID.hpp"

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

		// Receiving yaw and yaw rate feedback and normalizing between -1 and 1
		if (_vehicle_attitude_sub.updated()) {
   			_vehicle_attitude_sub.copy(&vehicle_attitude);
   			Vector3f rpy = getRPY(Quatf(vehicle_attitude.q));
			// Normalize yaw feedback between -pi and pi
			yaw_fb = rpy(2) / M_PIf;

			if (!module_initialization) {
			PX4_INFO("Initializing");
			yaw_input_integral = yaw_fb; // Initialize integral state to current yaw feedback
			module_initialization = true; // Set flag to true after initialization
		}
		}

		if (_vehicle_angular_velocity_sub.updated()) {
   			_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
			// Normalize yaw speed feedback between -MAX_YAW_SPEED and MAX_YAW_SPEED
	  		yaw_rate_fb = vehicle_angular_velocity.xyz[2]/ MAX_YAW_SPEED; 
 		}
		PX4_INFO("Y fb: %.2f, YR fb : %.2f", (double)yaw_fb, (double)yaw_rate_fb);


		// Compute control errors
		forward_euler_integration(dt, rc_input.roll); // Roll input as desired yaw
		float yaw_error = compute_yaw_error(yaw_input_integral, yaw_fb);
  		float yaw_rate_error = rc_input.roll - yaw_rate_fb;

		float yaw_control_input = K_p * yaw_rate_error  + K_i * std::copysign(1.0f, rc_input.roll) * yaw_error; // Proportional and integral control for yaw

		// Print throttle and yaw control inputs
		PX4_INFO("Throttle: %.2f, Roll input : %.2f, Yaw Control Input: %.2f", double(rc_input.throttle), (double)rc_input.roll, double(yaw_control_input));
		PX4_INFO("Y input integral: %.2f, Y error: %.2f, YR error : %.2f", (double)yaw_input_integral, (double)yaw_error, (double)yaw_rate_error);
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

void MarineNavigation::forward_euler_integration(const float &d_t, const float &u_n)
{
	float leak = 0.4f; // Leak factor for the state variable
	float epsilon = 0.1f;
	float x_n1 = yaw_input_integral + d_t* filterYawInput(u_n) - d_t * leak * (yaw_input_integral - yaw_fb); // Forward Euler integration

	PX4_INFO("x_n1 before wrapping: %.2f", (double)x_n1);
	if(x_n1 > 1) {
		x_n1 -= 2.0f; // Wrap around if exceeding pi
	} else if (x_n1 < -1) {
		x_n1 += 2.0f; // Wrap around if below -pi
	}

	if(fabs(compute_yaw_error(x_n1, yaw_fb)) < 1 - epsilon) {
		yaw_input_integral = x_n1; // Update state only if within valid range
		return;
	}
	else{
		return;
	}
}

float MarineNavigation::compute_yaw_error(const float &y_d, const float &y_fb)
{
	float error;
	// To go from the current to the desired yaw the rotattion can be done following two paths: short and long angle
	// Finding the short angle
	float e1 = y_d - y_fb; //Path1
	float e2 = -std::copysign(1.0f, e1) * (2.0f - std::fabs(e1)); //Path2
	if (fabs(e1) < fabs(e2)) {
		error = fabs(e1);
	} else {
		error = fabs(e2);
	}
	return error;
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

float MarineNavigation::filterYawInput(const float &yaw_input)
{
	if (yaw_input <= 0.01f && yaw_input >= -0.01f) {
		return 0.0f;	
	}
	else {
		return yaw_input; 
	}
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