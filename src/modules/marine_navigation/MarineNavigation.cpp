// CUSTOM MODE
#include "MarineNavigation.hpp"

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

	_vehicle_control_mode_sub.update(&_vehicle_control_mode);

	// Read RC stick input (throttle and roll)
	manual_control_setpoint_s rc_input{};

	if (_manual_control_sub.update(&rc_input) && _vehicle_control_mode.flag_control_prisma_marine_manual_enabled) {
		//	PX4_INFO("Throttle: %.2f | Roll: %.2f", (double)rc_input.throttle, (double)rc_input.roll);
		
		// ALlowing servos without armin
		actuator_armed_s armed_msg{};
		armed_msg.timestamp = hrt_absolute_time();
		armed_msg.armed = false;
		armed_msg.prearmed = true;
		armed_msg.manual_lockdown = false;
		armed_msg.force_failsafe = false;

		_armed_pub.publish(armed_msg);

		// Publish on actiuator_servo topic even if not armed
		actuator_servos_s actuator_servos{};
		actuator_servos.control[0] = rc_input.throttle; // Throttle
		actuator_servos.control[1] = rc_input.roll; // Roll
		_actuator_servos_pub.publish(actuator_servos);
		// Publish on orb_test topic	
	}
	// If NOT in manual control mode, stop the servos
	else if(!_vehicle_control_mode.flag_control_prisma_marine_manual_enabled) {
		actuator_servos_s actuator_servos{};
		actuator_servos.control[0] = 0; 
		actuator_servos.control[1] = 0; 
		_actuator_servos_pub.publish(actuator_servos);
		// Publish on orb_test topic	
	}

	perf_end(_loop_perf);
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