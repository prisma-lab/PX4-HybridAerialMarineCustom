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
	_fft_inv_cfg=kiss_fftr_alloc(FFT_SIZE, 1, nullptr, nullptr);
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
	//if (_armed && !_takeoff_sent) {
	//
	//	vehicle_command_s mode_cmd{};
	//	mode_cmd.timestamp = hrt_absolute_time();
	//	mode_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;

	//	mode_cmd.param1 = 1; // custom mode enabled
	//	mode_cmd.param2 = 4;
	//	mode_cmd.param3 = 2; // AUTO_TAKEOFF  Mode  for takeoff

	//	mode_cmd.target_system = 1;
	//	mode_cmd.target_component = 1;
	//	mode_cmd.source_system = 1;
	//	mode_cmd.source_component = 1;
	//	mode_cmd.from_external = false;

   	//	_vehicle_command_pub.publish(mode_cmd);

	//	PX4_INFO("Sent AUTO_TAKEOFF mode command");

	//	_takeoff_sent = true;

	//}



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


        // 2. create quaternion use synchronized data
	Quatf q(_last_attitude.q);

	// 3. create accelleration vector
	Vector3f acc_body(_sync_msg.accel[0], _sync_msg.accel[1], _sync_msg.accel[2]);

	// 4. Rotation from Body Frame to   World Frame

	Vector3f acc_world = q.rotateVector(acc_body);

	// 5. Gravity Compensation
	// PX4 follows the NED (North-East-Down) standard.
	// When at rest, the world-frame Z-acceleration (Down) is approximately -9.80665 m/s².
	// We add +9.80665f to center the signal at zero.
	// Note: In this centered signal, POSITIVE values indicate downward acceleration,
	// and NEGATIVE values indicate upward acceleration.
	float acc_z_pure = acc_world(2) + 9.80665f;

	// --- STEP 1: DECIMATION (Slow down from 250Hz to 25Hz) ---
//_skip_count++;
//if (_skip_count >= 10) {
 //   _skip_count = 0;

    const float* src_list[] = { &acc_z_pure };
    float* dst_list[] = { _buf_az };

    // This calls your updated circular logic:
    // It fills until max_rows, then starts shifting left.
    bool buffer_ready = zoh_utils::ZOHProcessor::bufferDataMulti(src_list, dst_list, 1, _fft_idx, FFT_SIZE);

    if (buffer_ready) {
        // --- STEP 2: COPY TO WORK BUFFER ---
        // CRITICAL: We copy _buf_az to work_buf so we don't ruin the
        // raw history with Mean Removal and Windowing.
        float work_buf[FFT_SIZE];
        float sum = 0.0f;

        for (size_t i = 0; i < FFT_SIZE; i++) {
            work_buf[i] = _buf_az[i];
            sum += work_buf[i];
        }

        // --- STEP 3: PRE-PROCESSING ---
        float mean = sum / (float)FFT_SIZE;
        for (size_t i = 0; i < FFT_SIZE; i++) {
            work_buf[i] -= mean;
            // Hanning Window to fix the spectral leakage (the 4.63m error)
            float window = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(FFT_SIZE - 1)));
            work_buf[i] *= window;
        }

        // --- STEP 4: FORWARD FFT ---
        float effective_fs = _actual_fs; // 10.0f; // 25.0 Hz
        float df = effective_fs / (float)FFT_SIZE;
        kiss_fftr(_fft_cfg, work_buf, _fft_out);

        // --- STEP 5: FREQUENCY DOMAIN INTEGRATION ---
        // Clean the integration buffer first
        for (size_t i = 0; i <= (FFT_SIZE / 2); i++) {
            _fft_integrated[i].r = 0.0f;
            _fft_integrated[i].i = 0.0f;
        }

        for (size_t bin_idx = 1; bin_idx <= (FFT_SIZE / 2); bin_idx++) {
            float freq = bin_idx * df;

            // General Guard: 0.05Hz to 1.5Hz covers most ocean swells
            if (freq > 0.05f && freq < 2.0f) {
                float omega = 2.0f * (float)M_PI * freq;
                float scale = -1.0f / (omega * omega); // Double integration (Accel -> Disp)

                _fft_integrated[bin_idx].r = _fft_out[bin_idx].r * scale;
                _fft_integrated[bin_idx].i = _fft_out[bin_idx].i * scale;
            }
        }

        // --- STEP 6: INVERSE FFT (Back to Time Domain) ---
        kiss_fftri(_fft_inv_cfg, _fft_integrated, _displacement_output);
	// This multiplier is key to matching your 2.3m simulation

	static int print_counter = 0;
	if (print_counter++ >= 100) { // Print every 40ms instead of every 4ms
	PX4_INFO("Heave: %4.2f", (double)_displacement_output[FFT_SIZE - 1]);

    }
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
