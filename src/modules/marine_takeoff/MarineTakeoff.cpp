#include "MarineTakeoff.hpp"

MarineTakeoff::MarineTakeoff() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

MarineTakeoff::~MarineTakeoff()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MarineTakeoff::init()
{


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



	         // ===========================
	// read   veichle_attitude and  IMU
	// ===========================
	const bool attitude_updated = _vehicle_attitude_sub.updated();
	const bool imu_status_updated = _vehicle_imu_status_sub.updated();

	// estimate frequency
	if (attitude_updated && _vehicle_attitude_sub.copy(&_last_attitude)) {

		_freq_att = _att_freq_est.estimate(_last_attitude.timestamp);
	}

	if (imu_status_updated && _vehicle_imu_status_sub.copy(&_last_imu_status)) {

		_freq_imu = _imu_freq_est.estimate(_last_imu_status.timestamp);
	}

	///  debug

	//	static uint64_t last_freq_print = 0;
        //      uint64_t now = hrt_absolute_time();
	//	if (now - last_freq_print > 1000000) { // 1 secondo
        //		PX4_INFO("--- Frequenze Topics ---");
	//		PX4_INFO("Attitude freq: %.2f Hz", (double)_freq_att);
	//		PX4_INFO("IMU Status freq: %.2f Hz", (double)_freq_imu);
	//		last_freq_print = now;
	//}




	// ===========================
	// ZOH + Sampling at fastest topic update
	// ===========================
	 zoh_utils::TopicSelector::Candidate streams[] = {
	{"ATT", _freq_att, attitude_updated},
	{"IMU", _freq_imu, imu_status_updated}
	};

	// 2. select fastest topic
	const zoh_utils::TopicSelector::Candidate* best = zoh_utils::TopicSelector::getBestCandidate(streams, 2);

	// 3. ZOH Trigger

        bool reference_updated = (best != nullptr) && best->updated;

        if (reference_updated) {



                // --- Synctonization  MarineSync ---
		_sync_msg.timestamp = hrt_absolute_time();

		//  save syncronized  data
		zoh_utils::ZOHHandler::synchronize(_last_attitude.q, _sync_msg.q, sizeof(_sync_msg.q));


		zoh_utils::ZOHHandler::synchronize(_last_imu_status.mean_accel, _sync_msg.accel, sizeof(_sync_msg.accel));

		_marine_sync_pub.publish(_sync_msg);

                // --- create buffer  BUFFER WITH  ZOH ---
                int temp_idx = idx; // Prepariamo il posto nella riga attuale


                zoh_utils::ZOHHandler::bufferData(_last_attitude.q, 4, (float*)q_buffer, temp_idx, FFT_SIZE);


                bool buffer_pieno = zoh_utils::ZOHHandler::bufferData(_last_imu_status.mean_accel, 3, (float*)accel_buffer, idx, FFT_SIZE);

                // --- PUNTO 2: CHECK SAVING  ---
                if (buffer_pieno) {

                        if (_is_logging) {
                                char file_path[128];
                                // USE  PX4_STORAGEDIR TO SAVE ON  SD card
                                snprintf(file_path, sizeof(file_path), PX4_STORAGEDIR "/log_%d.csv", _file_counter++);

                                FILE *csv_file = fopen(file_path, "w");

                                if (csv_file) {
                                        fprintf(csv_file, "timestamp,qw,qx,qy,qz,acc_x,acc_y,acc_z\n");

                                        for (int i = 0; i < FFT_SIZE; i++) {
                                                fprintf(csv_file, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                                                        (unsigned long long)_last_attitude.timestamp,
                                                        (double)q_buffer[i][0], (double)q_buffer[i][1],
                                                        (double)q_buffer[i][2], (double)q_buffer[i][3],
                                                        (double)accel_buffer[i][0], (double)accel_buffer[i][1],
                                                        (double)accel_buffer[i][2]);
                                        }

                                        fclose(csv_file);
                                        PX4_INFO("Saved: %s", file_path);
                                } else {
                                        PX4_ERR("Error! no file creation  %s", file_path);
                                }
                        }

                        //  reset INDEX
                        idx = 0;
                }


                perf_end(_loop_perf);
        }
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
    // 1. CHECK IF MODULE IS ACKTIVE
    if (!is_running()) {
        PX4_ERR("Il modulo non e' avviato! Scrivi prima: marine_takeoff start");
        return 1;
    }

    // 2. WRITE  "marine_takeoff start_log"  TO SAVE CSV
    if (!strcmp(argv[0], "start_log")) {
        get_instance()->_is_logging = true;
        PX4_INFO(">>>  CSV  RECORDING START !");
        return 0;
    }

    // 3. WRITE  "marine_takeoff stop_log" TO STOP SAVING CSV
    if (!strcmp(argv[0], "stop_log")) {
        get_instance()->_is_logging = false; //
        PX4_INFO(">>>  CSV RECORDING  STOP .");
        return 0;
    }

    //
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
