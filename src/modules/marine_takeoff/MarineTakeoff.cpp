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
		mode_cmd.param3 = 2; // AUTO_TAKEOFF  Mode avvia il navigator altrimenti non parte la modalita di takeoff

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
	// Lettura topic attitude e IMU
	// ===========================
	const bool attitude_updated = _vehicle_attitude_sub.updated();
	const bool imu_status_updated = _vehicle_imu_status_sub.updated();

	// Aggiorna e stima frequenza usando la libreria
	if (attitude_updated && _vehicle_attitude_sub.copy(&_last_attitude)) {
		// Chiamata alla libreria: passa il timestamp del messaggio
		_freq_att = _att_freq_est.estimate(_last_attitude.timestamp);
	}

	if (imu_status_updated && _vehicle_imu_status_sub.copy(&_last_imu_status)) {
		// Chiamata alla libreria
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
	// ZOH + Sampling ad ogni aggiornamento del topic più veloce
	// ===========================
	 TopicSelector::Candidate streams[] = {
	{"ATT", _freq_att, attitude_updated},
	{"IMU", _freq_imu, imu_status_updated}
	};

	// 2. Troviamo il riferimento migliore tramite la libreria
	const TopicSelector::Candidate* best = TopicSelector::getBestCandidate(streams, 2);

	// 3. Verifichiamo se dobbiamo campionare (ZOH Trigger)
	// Il trigger scatta solo se il topic più veloce ha ricevuto un nuovo dato
	bool reference_updated = (best != nullptr) && best->updated;

	if (reference_updated) {
		// Salvataggio buffer
		q_buffer[idx][0] = _last_attitude.q[0];
		q_buffer[idx][1] = _last_attitude.q[1];
		q_buffer[idx][2] = _last_attitude.q[2];
		q_buffer[idx][3] = _last_attitude.q[3];

		accel_buffer[idx][0] = _last_imu_status.mean_accel[0];
		accel_buffer[idx][1] = _last_imu_status.mean_accel[1];
		accel_buffer[idx][2] = _last_imu_status.mean_accel[2];





		idx++;
		if (idx >= FFT_SIZE) {

    // Controlliamo l'interruttore: se è 'true' (acceso), scriviamo il file
    if (_is_logging) {
        char file_path[128];
        // Creiamo il nome: log_0.csv, poi log_1.csv, ecc.
        snprintf(file_path, sizeof(file_path), "log_%d.csv", _file_counter++);

        FILE *csv_file = fopen(file_path, "w"); // "w" significa: crea un file nuovo pulito

        if (csv_file) {
            // --- QUESTA È L'INTESTAZIONE ---
            // Scriviamo i nomi delle colonne così Excel le capisce subito
            fprintf(csv_file, "timestamp,qw,qx,qy,qz,acc_x,acc_y,acc_z\n");

            // Ora scriviamo tutti i dati che abbiamo nel buffer
            for (int i = 0; i < FFT_SIZE; i++) {
                fprintf(csv_file, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    (unsigned long long)_last_attitude.timestamp,
                    (double)q_buffer[i][0], (double)q_buffer[i][1],
                    (double)q_buffer[i][2], (double)q_buffer[i][3],
                    (double)accel_buffer[i][0], (double)accel_buffer[i][1],
                    (double)accel_buffer[i][2]);
            }

            fclose(csv_file); // Chiudiamo il file (importantissimo!)
            PX4_INFO("Salvato: %s", file_path);
        } else {
            PX4_ERR("Errore! Non riesco a creare il file %s", file_path);
        }
    }

    // Anche se non abbiamo salvato, svuotiamo il buffer per ricominciare
    idx = 0;


}
		// Debug
		//static uint64_t last_print_time = 0;
		//uint64_t now = hrt_absolute_time();
		//if (now - last_print_time > 1000000) { // 1 secondo
		  // Nome e frequenza del topic di riferimento
		//float ref_freq = att_is_faster ? _freq_att : _freq_imu;
		//const char* ref_name = att_is_faster ? "Attitude" : "IMU_Status";

		//PX4_INFO("--- ZOH Alignment ---");
		//PX4_INFO("Ref Topic: %s (%.2f Hz)", ref_name, (double)ref_freq);

		// --- Stampa quaternion (q[4]) ---
		//PX4_INFO("q: [%.3f, %.3f, %.3f, %.3f]",
		//(double)_last_attitude.q[0],
		//(double)_last_attitude.q[1],
		//(double)_last_attitude.q[2],
		//(double)_last_attitude.q[3]);

		// --- Stampa accelerazione media (mean_accel[3]) ---
		//PX4_INFO("mean_accel: [%.3f, %.3f, %.3f]",
		//	(double)_last_imu_status.mean_accel[0],
		//	(double)_last_imu_status.mean_accel[1],
		//	(double)_last_imu_status.mean_accel[2]);

	//last_print_time = now;
	//}





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
    // 1. Controlliamo se il modulo è acceso, altrimenti non possiamo parlargli
    if (!is_running()) {
        PX4_ERR("Il modulo non e' avviato! Scrivi prima: marine_takeoff start");
        return 1;
    }

    // 2. Se scrivi "marine_takeoff start_log"
    if (!strcmp(argv[0], "start_log")) {
        get_instance()->_is_logging = true; // ACCENDIAMO l'interruttore
        PX4_INFO(">>> Registrazione CSV INIZIATA!");
        return 0;
    }

    // 3. Se scrivi "marine_takeoff stop_log"
    if (!strcmp(argv[0], "stop_log")) {
        get_instance()->_is_logging = false; // SPEGNIAMO l'interruttore
        PX4_INFO(">>> Registrazione CSV FERMATA.");
        return 0;
    }

    // Se scrivi qualcosa che non capisce
    return print_usage("Comando sconosciuto! Usa: start_log o stop_log");

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
