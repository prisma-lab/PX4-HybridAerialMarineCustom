#include "MarineTakeoff.hpp"

MarineTakeoff::MarineTakeoff() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

MarineTakeoff::~MarineTakeoff()
{
	if (_fft_cfg) {
                free(_fft_cfg);
        }
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MarineTakeoff::init()
{

        _fft_cfg = kiss_fftr_alloc(FFT_SIZE, 0, nullptr, nullptr);

        if (_fft_cfg == nullptr) {
                PX4_ERR("FFT allocation failed!");
                return false;
        }
	// alternatively, Run on fixed interval
	 ScheduleOnInterval(2_ms);

	return true;
}

void MarineTakeoff::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}


	// Example
	//  grab latest accelerometer data
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}

	_vehicle_control_mode_sub.update(&_vehicle_control_mode);

	//if(_vehicle_control_mode.flag_control_marine_takeoff_enabled){
	//	PX4_INFO("Marine Takeoff mode enabled");
	//
	 if (!_armed) {
		// Arm the vehicle
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
		cmd.param1 = 1.0f; // 1 = arm
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd.source_system = 1;
		cmd.source_component = 1;
		cmd.from_external = false;

		_vehicle_command_pub.publish(cmd);
		PX4_INFO("Sent ARM command from MarineTakeoff module");
	}





	// If armed but not yet taken off
	if (_armed && !_takeoff_sent) {

		vehicle_command_s mode_cmd{};
		mode_cmd.timestamp = hrt_absolute_time();
		mode_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;

		mode_cmd.param1 = 1; // custom mode enabled
		mode_cmd.param2 = 4;
		mode_cmd.param3 = 2; // AUTO_TAKEOFF  Mode  for takeoff

		mode_cmd.target_system = 1;
		mode_cmd.target_component = 1;
		mode_cmd.source_system = 1;
		mode_cmd.source_component = 1;
		mode_cmd.from_external = false;

   		_vehicle_command_pub.publish(mode_cmd);

		PX4_INFO("Sent AUTO_TAKEOFF mode command");

		_takeoff_sent = true;

	}



        // ======================================================
	  //  UPDATE  VEHICLE_ATTITUDE AND VEHICLE_IMU_STATUS
	// ======================================================


        const bool attitude_updated = _vehicle_attitude_sub.updated();
	const bool imu_status_updated = _vehicle_imu_status_sub.updated();

	if (attitude_updated) {
		_vehicle_attitude_sub.copy(&_last_attitude);
	}

	if (imu_status_updated) {
		_vehicle_imu_status_sub.copy(&_last_imu_status);
	}

       // ======================================================
	// ZOH + Sampling at fastest topic update
	// ======================================================
        zoh_utils::ZOHProcessor::Input streams[] = {
		{_last_attitude.q, _last_attitude.timestamp, sizeof(_last_attitude.q), attitude_updated},
		{_last_imu_status.mean_accel, _last_imu_status.timestamp, sizeof(_last_imu_status.mean_accel), imu_status_updated}
	};

	auto result = _zoh_processor.process(streams, 2);

	if (result.updated) {
        // Sync data
        zoh_utils::ZOHProcessor::synchronize(_last_imu_status.mean_accel, _sync_msg.accel, sizeof(_sync_msg.accel));
        _marine_sync_pub.publish(_sync_msg);


        // 2. Creazione del Quaternione (usa i dati dell'attitude sincronizzata)
	Quatf q(_last_attitude.q);

	// 3. Vettore accelerazione dal messaggio sincronizzato
	Vector3f acc_body(_sync_msg.accel[0], _sync_msg.accel[1], _sync_msg.accel[2]);

	// 4. Rotazione nel World Frame
	// In Matrix, q.rotateVector(v) ruota il vettore dal Body al World frame
	Vector3f acc_world = q.rotateVector(acc_body);

	// 5. Compensazione Gravità
	// PX4 usa lo standard NED (North-East-Down).
	// Se il drone è fermo, acc_world(2) (Z-down) è circa -9.8 m/s^2.
	// Sommiamo 9.80665f per centrare il segnale sullo zero.
	float acc_z_pure = acc_world(2) + 9.80665f;

	// 6. Riempimento Buffer FFT

	const float* src_list[] = { &acc_z_pure };
	float* dst_list[] = { _buf_az };

	// Ora passiamo gli array alla funzione. '1' indica che stiamo passando un solo segnale.
	bool full = zoh_utils::ZOHProcessor::bufferDataMulti(src_list, dst_list, 1, _fft_idx, FFT_SIZE);


        if (full) {
                // MAGIC: Perform FFT on Z Acceleration
                kiss_fftr(_fft_cfg, _buf_az, _fft_out);

                // Print the result (DC component = average of the signal)
                float dc_offset = _fft_out[0].r / FFT_SIZE;
                PX4_INFO("FFT DONE! Avg Accel Z: %.3f", (double)dc_offset);

                _fft_idx = 0; // Reset
        }
    }






                perf_end(_loop_perf);
        //}
}

int MarineTakeoff::task_spawn(int argc, char *argv[])
{
	MarineTakeoff *instance = new MarineTakeoff();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MarineTakeoff::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MarineTakeoff::custom_command(int argc, char *argv[])
{

    return print_usage("UNKNOWN COMMAND ! Use: start_log o stop_log");

}

int MarineTakeoff::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int marine_takeoff_main(int argc, char *argv[])
{
	return MarineTakeoff::main(argc, argv);
}
