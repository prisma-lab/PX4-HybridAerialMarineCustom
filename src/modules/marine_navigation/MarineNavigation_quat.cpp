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
		// Update parametrs variables
		Prop_C = _param_marine_prop_c.get();
		K_q = _param_marine_kq.get();
		K_r = _param_marine_kr.get();
		K_t = _param_marine_kt.get();
		K_t_ff = _param_marine_kt_ff.get();
		leak = _param_marine_leak.get();
		drag_coeff = _param_marine_drag.get();
		max_propeller_speed = _param_marine_max_speed.get();
		max_yawspeed = _param_marine_max_yaw.get();
		left_th_x = _param_marine_l_x.get();
		left_th_y = _param_marine_l_y.get();
		right_th_x = _param_marine_r_x.get();
		right_th_y = _param_marine_r_y.get();
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

	if (_manual_control_sub.updated()) {
		_manual_control_sub.copy(&rc_input);
	}

	if (_pos_sp_triplet_sub.updated()) {
		PX4_INFO("Received new position setpoint triplet");
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
		// PRINT LAT LON
		//PX4_INFO("Current WP: lat %.7f, lon %.7f", (double)_pos_sp_triplet.current.lat, (double)_pos_sp_triplet.current.lon);
	}

	in_marine_mode = (vehicle_control_mode.flag_control_prisma_marine_manual_enabled || vehicle_control_mode.flag_control_prisma_marine_manual_ts_enabled || 
		vehicle_control_mode.flag_control_prisma_marine_manual_ff_enabled || vehicle_control_mode.flag_control_prisma_auto_marine_enabled);

	Vector3f rpy;
	float force_input;
	float torque_input;

	if (vehicle_control_mode.flag_control_prisma_marine_manual_enabled) {
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
		
		// Update quaternion desired based on angular velocity input
		updateQDesired(dt, computeOmegaInput(rc_input.roll)); 
		// Compute quaternion error
		quat_error = quat_d * quat_fb.inversed();
		quat_error.normalize();
		if (fabs(quat_error(3)) < 0.01f){
			quat_error(3) = 0.0f;
		}
		// Compute omega desired and deal with unwinding
		Vector3f omega_d = Vector3f(quat_error(1), quat_error(2), quat_error(3)) * K_q * std::copysign(1.0f, quat_error(0));
		if (fabs(omega_d(2)) < 0.01f) {
			omega_d(2) = 0.0f;
		}
		torque_input = K_r * (-omega_d(2) + yaw_rate_fb);

		// Computing forward force input
		if(_vehicle_local_position_sub.updated()) {
			_vehicle_local_position_sub.copy(&vehicle_local_position);
			v_x = vehicle_local_position.vx * cos(rpy(2)) + vehicle_local_position.vy * sin(rpy(2)); // Forward velocity in m/s
			if (fabs(v_x) < 0.09f) {
				v_x = 0.0f;
			}
		}

		if(vehicle_local_position.v_xy_valid)
		{
			force_input = drag_coeff* v_x * fabs(v_x) + K_t * (rc_input.throttle * max_propeller_speed - v_x); // Proportional controller on forward velocity
		}
		else {
			PX4_INFO("Velocity feedback not valid, switch to feedforward");
			mavlink_log_critical(&_mavlink_log_pub, "Velocity feedback lost, switching to feedforward velocity control\t");
        		events::send(events::ID("velocity_feedback_lost"), events::Log::Alert, "Velocity feedback lost, switching to feedforward velocity control");
			force_input = K_t_ff * (rc_input.throttle * max_propeller_speed); // Feedforward only if velocity not valid

			// Resetting ekf
			vehicle_command_s cmd{};
			cmd.timestamp = hrt_absolute_time();
			cmd.param1 = 1.0f;
			cmd.command = 179; // MAV_CMD_DO_SET_MODE
			cmd.target_system = 1;
			cmd.target_component = 1;
			cmd.source_system = 1;
			cmd.source_component = 1;
			cmd.from_external = true;

   			_vehicle_command_pub.publish(cmd);
		}

		// Computing control inputs for the two propellers
		control_input = getControlInput(force_input, torque_input);

		// Publish on actuator_servos topic even if not armed
		// actuator_servos_s actuator_servos{};
		// actuator_servos.timestamp = hrt_absolute_time();
		// actuator_servos.control[0] = control_input(0); // Left propeller control input
		// actuator_servos.control[1] = control_input(1); // Right propeller
		// _actuator_servos_pub.publish(actuator_servos);	

		// Publish on marine_navigation topic
		//marine_navigation_s marine_navigation{};
		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.q_desired[0] = quat_d(0);
		marine_navigation.q_desired[1] = quat_d(1);
		marine_navigation.q_desired[2] = quat_d(2);
		marine_navigation.q_desired[3] = quat_d(3);
		marine_navigation.q_feedback[0] = quat_fb(0);
		marine_navigation.q_feedback[1] = quat_fb(1);
		marine_navigation.q_feedback[2] = quat_fb(2);
		marine_navigation.q_feedback[3] = quat_fb(3);
		marine_navigation.q_error[0] = quat_error(0);
		marine_navigation.q_error[1] = quat_error(1);
		marine_navigation.q_error[2] = quat_error(2);
		marine_navigation.q_error[3] = quat_error(3);
		marine_navigation.desired_speed = rc_input.throttle * max_propeller_speed;
		marine_navigation.desired_angular_vel = computeOmegaInput(rc_input.roll);
		marine_navigation.speed_error = rc_input.throttle * max_propeller_speed - v_x;
		marine_navigation.angular_vel_error = computeOmegaInput(rc_input.roll) - yaw_rate_fb;
		marine_navigation.omega_desired_z = omega_d(2);
		marine_navigation.force_input = force_input;
		marine_navigation.torque_input = torque_input;
		marine_navigation.angular_error = 2.0f * acosf(fabsf(quat_error(0)));

		//_marine_navigation_pub.publish(marine_navigation);

		// PX4_INFO printing
		PX4_INFO("Y fb: %.2f, YR fb: %.2f, Vx fb: %.2f", (double)rpy(2), (double)yaw_rate_fb, (double)v_x);
		PX4_INFO("Vx input: %.2f, Omega z input: %.2f", (double)(rc_input.throttle*max_propeller_speed), (double)computeOmegaInput(rc_input.roll));
		PX4_INFO("Vx error: %.2f, Omega z error: %.2f", (double)(rc_input.throttle*max_propeller_speed - v_x), (double)(computeOmegaInput(rc_input.roll) - yaw_rate_fb));
		PX4_INFO("quat_d [0]: %.2f, quat_fb [0]: %.2f", (double)quat_d(0), (double)quat_fb(0));
		PX4_INFO("quat_d [3]: %.2f, quat_fb [3]: %.2f", (double)quat_d(3), (double)quat_fb(3));
		PX4_INFO("Quat error [0]: %.2f, Quat error [3]: %.2f, YR error : %.2f", (double)quat_error(0), (double)quat_error(3), (double)(omega_d(2) - yaw_rate_fb));
		PX4_INFO("Omega d: %.2f, %.2f, %.2f", (double)omega_d(0), (double)omega_d(1), (double)omega_d(2));
		PX4_INFO("angular error: %.2f", (double)(2.0f * acosf(fabsf(quat_error(0)))));
		PX4_INFO("Throttle: %.2f, Roll input : %.2f, Force input: %.2f, Torque Input: %.2f", (double)rc_input.throttle, (double)rc_input.roll, (double)force_input, (double)torque_input);
		PX4_INFO("Control Input Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));
	}
	else if (vehicle_control_mode.flag_control_prisma_marine_manual_ts_enabled) {
		// Similar implementation for TS control mode can be added here

		float thrust = rc_input.throttle;
		float steering = rc_input.roll;
		
		if (fabs(thrust) < 0.09f) thrust = 0.0f;
		if (fabs(steering) < 0.09f) steering = 0.0f;

		if (fabs(steering) < 0.01f)
		{
			control_input(0) = thrust;
			control_input(1) = thrust;
		}
		else if (steering > 0.01f)
		{
			control_input(0) = thrust;
			control_input(1) = -(thrust +1)*steering +1;
		} else {
			control_input(0) = (thrust +1)*steering +1;
			control_input(1) = thrust;
		}

		// Print joystick input and controls
		PX4_INFO("Joystick Input - Throttle: %.2f, Roll: %.2f", (double)rc_input.throttle, (double)rc_input.roll);
		PX4_INFO("Control Input - Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));

		//marine_navigation_s marine_navigation{};
		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.q_desired[0] = 0;
		marine_navigation.q_desired[1] = 0;
		marine_navigation.q_desired[2] = 0;
		marine_navigation.q_desired[3] = 0;
		marine_navigation.q_feedback[0] = 0;
		marine_navigation.q_feedback[1] = 0;
		marine_navigation.q_feedback[2] = 0;
		marine_navigation.q_feedback[3] = 0;
		marine_navigation.q_error[0] = 0;
		marine_navigation.q_error[1] = 0;
		marine_navigation.q_error[2] = 0;
		marine_navigation.q_error[3] = 0;
		marine_navigation.desired_speed = 0;
		marine_navigation.desired_angular_vel = 0;
		marine_navigation.speed_error = 0;
		marine_navigation.angular_vel_error = 0;
		marine_navigation.omega_desired_z = 0;
		marine_navigation.force_input = 0;
		marine_navigation.torque_input = 0;
		marine_navigation.angular_error = 0;

		// _marine_navigation_pub.publish(marine_navigation);

		// Reset variables
		if (module_initialization) {
			module_initialization = false; // Reset initialization flag
		}
	}
	else if (vehicle_control_mode.flag_control_prisma_marine_manual_ff_enabled) {

		if(fabs(rc_input.throttle) < 0.09f) control_input(0) = 0.0f;
		if(fabs(rc_input.pitch) < 0.09f) control_input(1) = 0.0f;
		else control_input = Vector2f(rc_input.throttle, rc_input.pitch);
		
		PX4_INFO("Control Input - Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));

		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.q_desired[0] = 0;
		marine_navigation.q_desired[1] = 0;
		marine_navigation.q_desired[2] = 0;
		marine_navigation.q_desired[3] = 0;
		marine_navigation.q_feedback[0] = 0;
		marine_navigation.q_feedback[1] = 0;
		marine_navigation.q_feedback[2] = 0;
		marine_navigation.q_feedback[3] = 0;
		marine_navigation.q_error[0] = 0;
		marine_navigation.q_error[1] = 0;
		marine_navigation.q_error[2] = 0;
		marine_navigation.q_error[3] = 0;
		marine_navigation.desired_speed = 0;
		marine_navigation.desired_angular_vel = 0;
		marine_navigation.speed_error = 0;
		marine_navigation.angular_vel_error = 0;
		marine_navigation.omega_desired_z = 0;
		marine_navigation.force_input = 0;
		marine_navigation.torque_input = 0;
		marine_navigation.angular_error = 0;
	}
	else if(vehicle_control_mode.flag_control_prisma_auto_marine_enabled && _pos_sp_triplet.current.valid) {
		// if (_vehicle_attitude_sub.updated()) {
		// 		_vehicle_attitude_sub.copy(&vehicle_attitude);
		// }

		// if (_vehicle_angular_velocity_sub.updated()) {
		// 		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		// }

		// if (_vehicle_local_position_sub.updated()) {
		// 		_vehicle_local_position_sub.copy(&vehicle_local_position);
		// }

		// if (!_l1_initialized) {
		// 		_l1_pos_ctrl.set_l1_damping(3.75f);
		// 		_l1_pos_ctrl.set_l1_period(3.0f);
		// 		_l1_initialized = true;
		// }

		// if (vehicle_local_position.xy_global
		// 	&& (!_global_local_proj_ref.isInitialized()
		// 		|| _global_local_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
		// 		_global_local_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
		// 												vehicle_local_position.ref_timestamp);
		// }

		// if (!_global_local_proj_ref.isInitialized() || !vehicle_local_position.xy_valid || !vehicle_local_position.v_xy_valid) {
		// 		PX4_WARN("Auto marine: local position not ready");
		// 		control_input = Vector2f(0.0f, 0.0f);
		// 		marine_navigation = {};
		// 		marine_navigation.timestamp = hrt_absolute_time();
		// } else {
		// 		Vector3f rpy = getRPY(Quatf(vehicle_attitude.q));
		// 		yaw_rate_fb = vehicle_angular_velocity.xyz[2];

		// 		Vector2f curr_pos_local{vehicle_local_position.x, vehicle_local_position.y};
		// 		Vector2f ground_speed{vehicle_local_position.vx, vehicle_local_position.vy};

		// 		Vector2f curr_wp_local = _global_local_proj_ref.project(_pos_sp_triplet.current.lat,
		// 												_pos_sp_triplet.current.lon);
		// 		Vector2f prev_wp_local = curr_wp_local;

		// 		if (_pos_sp_triplet.previous.valid) {
		// 				prev_wp_local = _global_local_proj_ref.project(_pos_sp_triplet.previous.lat,
		// 												_pos_sp_triplet.previous.lon);
		// 		}

		// 		_l1_pos_ctrl.navigate_waypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed);

		// 		double current_lat{};
		// 		double current_lon{};
		// 		_global_local_proj_ref.reproject(curr_pos_local(0), curr_pos_local(1), current_lat, current_lon);
		// 		float dist_target = get_distance_to_next_waypoint(current_lat, current_lon,
		// 							_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

		// 		float desired_speed = _v_cruise_auto;

		// 		if (dist_target < _d_slow_auto) {
		// 				desired_speed = _v_cruise_auto * (dist_target / _d_slow_auto);
		// 				desired_speed = math::max(desired_speed, 0.0f);
		// 		}

		// 		float heading_error = wrap(_l1_pos_ctrl.nav_bearing() - rpy(2));

		// 		// if (fabs(heading_error) > 0.7f) {
		// 		// 	desired_speed = 0.0f;
		// 		// }

		// 		v_x = vehicle_local_position.vx * cos(rpy(2)) + vehicle_local_position.vy * sin(rpy(2));
				
				

				

		// 		//float torque_input = _Kp_psi_auto * heading_error - _Kd_psi_auto * yaw_rate_fb;

		// 		float yaw_rate_cmd = _l1_pos_ctrl.nav_lateral_acceleration_demand() / math::max(v_x, 0.1f);

		// 		yaw_rate_cmd = math::constrain(yaw_rate_cmd, -max_yawspeed, max_yawspeed);

		// 		float yaw_rate_input = 6.0f * (yaw_rate_cmd - yaw_rate_fb);

		// 		if (fabs(yaw_rate_cmd - yaw_rate_fb) > 0.8f) {
		// 		 	desired_speed = 0.03f;
		// 		}

		// 		float speed_error = desired_speed - v_x;

		// 		_int_v_err_auto = math::constrain(_int_v_err_auto + speed_error * dt, -_v_cruise_auto, _v_cruise_auto);

		// 		float force_input = drag_coeff * v_x * fabsf(v_x) + K_t * speed_error; //+ _int_v_err_auto;

		// 		control_input = getControlInput(force_input, yaw_rate_input);

		// 		// PRINT force input and yaw rate cmd
		// 		PX4_INFO("Force input: %.2f, Yaw rate cmd: %.2f", (double)force_input, (double)yaw_rate_cmd);
		// 		// PRint desired speed and speed error
		// 		PX4_INFO("Desired speed: %.2f, Speed error: %.2f", (double)desired_speed, (double)speed_error);
		// 		// Print heading error
		// 		PX4_INFO("Heading error: %.2f", (double)heading_error);

		// 		marine_navigation.timestamp = hrt_absolute_time();
		// 		marine_navigation.q_desired[0] = 0;
		// 		marine_navigation.q_desired[1] = 0;
		// 		marine_navigation.q_desired[2] = 0;
		// 		marine_navigation.q_desired[3] = 0;
		// 		marine_navigation.q_feedback[0] = 0;
		// 		marine_navigation.q_feedback[1] = 0;
		// 		marine_navigation.q_feedback[2] = 0;
		// 		marine_navigation.q_feedback[3] = 0;
		// 		marine_navigation.q_error[0] = 0;
		// 		marine_navigation.q_error[1] = 0;
		// 		marine_navigation.q_error[2] = 0;
		// 		marine_navigation.q_error[3] = 0;
		// 		marine_navigation.desired_speed = desired_speed;
		// 		marine_navigation.desired_angular_vel = 0;//heading_error;
		// 		marine_navigation.speed_error = speed_error;
		// 		marine_navigation.angular_vel_error = 0;//heading_error;
		// 		marine_navigation.omega_desired_z = _l1_pos_ctrl.nav_lateral_acceleration_demand();
		// 		marine_navigation.force_input = force_input;
		// 		marine_navigation.torque_input = 0;//torque_input;
		// 		marine_navigation.angular_error = 0;//heading_error;
		// }
		// Aggiornamento attitude e rate (come prima)
		// Aggiornamento attitude e rate
		if (_vehicle_attitude_sub.updated()) {
			_vehicle_attitude_sub.copy(&vehicle_attitude);
			 rpy = getRPY(Quatf(vehicle_attitude.q));
		}	

		if (_vehicle_angular_velocity_sub.updated()) {
			_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		}

		if (_vehicle_local_position_sub.updated()) {
			_vehicle_local_position_sub.copy(&vehicle_local_position);
		}

		// Inizializza reference globale (lat/lon -> locale) se necessario
		if (vehicle_local_position.xy_global
			&& (!_global_local_proj_ref.isInitialized()
			|| _global_local_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {

			_global_local_proj_ref.initReference(vehicle_local_position.ref_lat,
								vehicle_local_position.ref_lon,
								vehicle_local_position.ref_timestamp);
		}

		if (!_global_local_proj_ref.isInitialized() || !vehicle_local_position.xy_valid || !vehicle_local_position.v_xy_valid) {
			PX4_WARN("Auto marine PF-FL: local position not ready");
			control_input = Vector2f(0.0f, 0.0f);
			marine_navigation = {};
			marine_navigation.timestamp = hrt_absolute_time();
			return;
		}

		// ----- PARAMETRI CONTROLLER (da portare idealmente a parametri PX4) -----
		const float l_pf  = 1.5f;   // distanza hand-point in avanti [m]
		const float k_px  = 0.5f;
		const float k_py  = 0.5f;
		const float k_vx  = 1.0f;
		const float k_vy  = 1.0f;
		const float k_Ix  = 0.0f;   // per ora niente integrale (corrente non modellata)
		const float k_Iy  = 0.0f;
		//const float max_yawspeed_cmd = max_yawspeed; // già definito altrove

		// Stati di memoria del path-following: parametro s e integrali
		static float s_pf = 0.0f;   // parametro lungo il segmento [m]
		static float xi1_I = 0.0f;  // integrale errore posizione hand-point x
		static float xi2_I = 0.0f;  // integrale errore posizione hand-point y

		// ----- ESTRAZIONE STATI -----
		const float yaw = rpy(2);
		yaw_rate_fb = vehicle_angular_velocity.xyz[2];

		Vector2f curr_pos_local{vehicle_local_position.x, vehicle_local_position.y};
		Vector2f ground_speed{vehicle_local_position.vx, vehicle_local_position.vy};

		// Waypoint corrente e precedente (segmento rettilineo in locale)
		Vector2f curr_wp_local = _global_local_proj_ref.project(_pos_sp_triplet.current.lat,
									_pos_sp_triplet.current.lon);
		Vector2f prev_wp_local = curr_wp_local;

		if (_pos_sp_triplet.previous.valid) {
			prev_wp_local = _global_local_proj_ref.project(_pos_sp_triplet.previous.lat,
									_pos_sp_triplet.previous.lon);
		}

		Vector2f path_vec = curr_wp_local - prev_wp_local;
		float path_len = path_vec.norm();

		if (path_len < 1e-3f) {
			PX4_WARN("Auto marine PF-FL: path segment too short");
			control_input = Vector2f(0.0f, 0.0f);
			return;
		}

		Vector2f t_hat = path_vec / path_len; // tangente unitaria

		// ----- HAND POINT h = (xi1, xi2) -----
		const float x  = curr_pos_local(0);
		const float y  = curr_pos_local(1);

		const float cos_yaw = cosf(yaw);
		const float sin_yaw = sinf(yaw);

		const float xi1 = x + l_pf * cos_yaw;
		const float xi2 = y + l_pf * sin_yaw;

		// ----- PUNTO DI RIFERIMENTO γ(s) SUL PATH -----
		// s_pf è la distanza lungo il segmento [0, path_len]
		// Punto sul path:
		const float x_gamma = prev_wp_local(0) + t_hat(0) * s_pf;
		const float y_gamma = prev_wp_local(1) + t_hat(1) * s_pf;

		// Errore hand-point rispetto al path (per legge su s)
		const float tilde_xi1 = xi1 - x_gamma;
		const float tilde_xi2 = xi2 - y_gamma;
		const float e_h_sq    = tilde_xi1 * tilde_xi1 + tilde_xi2 * tilde_xi2;

		// ----- PROFILO DI VELOCITÀ LUNGO IL PATH (tipo slowdown vicino al WP) -----
		double current_lat{};
		double current_lon{};
		_global_local_proj_ref.reproject(curr_pos_local(0), curr_pos_local(1), current_lat, current_lon);

		const float dist_target = get_distance_to_next_waypoint(current_lat, current_lon,
									_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

		float U_cmd = _v_cruise_auto;

		if (dist_target < _d_slow_auto) {
			U_cmd = _v_cruise_auto * (dist_target / _d_slow_auto);
			U_cmd = math::max(U_cmd, 0.0f);
		}

		// ----- DINAMICA DEL PARAMETRO s (come eq. (60) del paper) -----
		const float s_dot = U_cmd * (1.0f - tanhf(e_h_sq));

		s_pf += s_dot * dt;
		s_pf = math::constrain(s_pf, 0.0f, path_len);

		// Aggiorna il punto di riferimento γ(s_pf)
		const float x_d = prev_wp_local(0) + t_hat(0) * s_pf;
		const float y_d = prev_wp_local(1) + t_hat(1) * s_pf;

		// Derivate spaziali per il segmento: d x/ds = t_hat(0), d y/ds = t_hat(1)
		const float x_s = t_hat(0);
		const float y_s = t_hat(1);

		// Velocità desiderata del punto sul path: x_dot(s), y_dot(s)
		const float x_dot_d = s_dot * x_s;
		const float y_dot_d = s_dot * y_s;

		// Feedforward di curvatura (per segmento rettilineo è zero)
		const float x_ddot_star = 0.0f;
		const float y_ddot_star = 0.0f;

		// ----- VELOCITÀ HAND POINT (xi3, xi4) -----
		const float x_dot = vehicle_local_position.vx;
		const float y_dot = vehicle_local_position.vy;

		const float xi1_dot = x_dot - l_pf * sin_yaw * yaw_rate_fb;
		const float xi2_dot = y_dot + l_pf * cos_yaw * yaw_rate_fb;

		// Nel paper xi3, xi4 sono velocità relative alla corrente: qui assumiamo corrente ≈ 0
		const float xi3 = xi1_dot;
		const float xi4 = xi2_dot;

		// ----- ERRORI PER IL CONTROLLO (stile eq. 59a, 59b) -----
		const float e1 = xi1 - x_d;
		const float e2 = xi2 - y_d;
		const float e3 = xi3 - x_dot_d;
		const float e4 = xi4 - y_dot_d;

		// Integrali errore posizione (implementazione ∫ e dt)
		xi1_I += e1 * dt;
		xi2_I += e2 * dt;

		// ----- CONTROLLO VIRTUALE μ (PATH FOLLOWING) -----
		float mu1 = -k_vx * e3 - k_px * e1 - k_Ix * xi1_I + x_ddot_star;
		float mu2 = -k_vy * e4 - k_py * e2 - k_Iy * xi2_I + y_ddot_star;


		//-------------------------------------------------------------------
		// // Expressng acceleration commands in body frame
		// // Rotation matrix from NED to body frame
		// // R_nb = [ cosψ sinψ
		// //         -sinψ cosψ ]
		// SquareMatrix<float, 2> R_nb;
		// R_nb(0,0) =  cos_yaw;    R_nb(0,1) =  sin_yaw;
		// R_nb(1,0) = -sin_yaw;    R_nb(1,1) =  cos_yaw;
		// // Vettore μ in body frame
		// float mu_b_1 = R_nb(0,0) * mu1 + R_nb(0,1) * mu2;
		// float mu_b_2 = R_nb(1,0) * mu1 + R_nb(1,1) * mu2;

		// // PRINT mu_b_1 e mu_b_2
		// PX4_INFO("mu_b_1: %.2f, mu_b_2: %.2f", (double)mu_b_1, (double)mu_b_2);

		// // Computing desired vforward velocity (surge) integrating (with leakage) linear acceleration (mu_b_1)
		// if(_vehicle_local_position_sub.updated())
		// {
		// 	_vehicle_local_position_sub.copy(&vehicle_local_position);
		// }
		// if(vehicle_local_position.v_xy_valid)
		// {
		// 	v_x = vehicle_local_position.vx * cos(rpy(2)) + vehicle_local_position.vy * sin(rpy(2)); // Forward velocity in m/s
		// 	vforward_cmd += mu_b_1 * dt - leak * dt * (vforward_cmd - v_x);
		// 	// Constrain to max propeller speed
		// 	vforward_cmd = math::constrain(vforward_cmd, -max_propeller_speed, max_propeller_speed);
		// 	force_input = drag_coeff* v_x * fabs(v_x) + 0.3f * K_t * (vforward_cmd - v_x); // Proportional controller on forward velocity
		// 	// PRINT forward vlocity command, feddback linear velocity and force input
		// 	PX4_INFO("Vforward cmd: %.2f, Vforward fb: %.2f, Force input: %.2f", (double)vforward_cmd, (double)v_x, (double)force_input);
		// }
		// else
		// {
		// 	PX4_INFO("Velocity feedback not valid, switch to feedforward");
		// 	mavlink_log_critical(&_mavlink_log_pub, "Velocity feedback lost, switching to feedforward velocity control\t");
		// 		events::send(events::ID("velocity_feedback_lost_auto_navigation"), events::Log::Alert, "Velocity feedback lost, switching to feedforward velocity control");
		// 	force_input = K_t_ff * (rc_input.throttle * max_propeller_speed); // Feedforward only if velocity not valid

		// 	// Resetting ekf
		// 	vehicle_command_s cmd{};
		// 	cmd.timestamp = hrt_absolute_time();
		// 	cmd.param1 = 1.0f;
		// 	cmd.command = 179; // MAV_CMD_DO_SET_MODE
		// 	cmd.target_system = 1;
		// 	cmd.target_component = 1;
		// 	cmd.source_system = 1;
		// 	cmd.source_component = 1;
		// 	cmd.from_external = true;

		// 	_vehicle_command_pub.publish(cmd);

		// 	force_input = K_t_ff * mu_b_1; // Feedforward only if velocity not valid
		// }

		// if (fabs(mu_b_2) < 0.01f) {
		// 	// avoid very small mu_b_2
		// 	mu_b_2 = 0.0f;
		// }
		// // float omega_desired_z = mu_b_2 / math::max(fabsf(vforward_cmd), 0.1f); // avoid division by zero
		// // omega_desired_z = math::constrain(omega_desired_z, -max_yawspeed, max_yawspeed);

		// // torque_input = 0.9f * K_r * (omega_desired_z - yaw_rate_fb);

		// float omega_rate = (mu_b_2 - vforward_cmd * omega_d_cmd) / l_pf;
		// omega_d_cmd += omega_rate * dt - leak * dt * (omega_d_cmd - yaw_rate_fb);
		// omega_d_cmd = math::constrain(omega_d_cmd, -max_yawspeed, max_yawspeed);

		// torque_input = 0.4f * K_r * (omega_d_cmd - yaw_rate_fb) + 0.5f * omega_rate;

		// // PRINT desired angular speed , feedback angular speed and torque input
		// PX4_INFO("Omega desired z: %.2f, Yaw rate feedback: %.2f, Torque input: %.2f", (double)omega_d_cmd, (double)yaw_rate_fb, (double)torque_input);

		// // Print position of hand frame and reference point on path
		// PX4_INFO("Hand point xi1: %.2f, xi2: %.2f, Path point x_d: %.2f, y_d: %.2f", (double)xi1, (double)xi2, (double)x_d, (double)y_d);

		// control_input = getControlInput(force_input, torque_input);
	
		// // Computing desired angular speed based on tangential acceleration command (mu_b_2)
		//-------------------------------------------------------------------


		// ----- FEEDBACK LINEARIZATION: μ → (τ_u, τ_r) -----
		// μ = [mu1; mu2] è l'accelerazione desiderata dell'hand-point nel frame NED.
		// Dal paper (eq. 12): [τ_u; τ_r] = R_h(ψ)^(-1) ( μ - F_ξ )
		//
		// R_h(ψ) = [ cosψ   -l sinψ
		//            sinψ    l cosψ ]
		//
		// Qui implementiamo la parte R_h(ψ)^(-1). Per ora assumiamo F_ξ ≈ 0 (kinematic FL).
		// TODO: se vuoi la FL dinamica completa, inserisci qui F_xi3,F_xi4 dal tuo modello idrodinamico.

		SquareMatrix<float, 2> Rh;
		Rh(0,0) =  cos_yaw;    Rh(0,1) = -l_pf * sin_yaw;
		Rh(1,0) =  sin_yaw;    Rh(1,1) =  l_pf * cos_yaw;

		//Vettore μ
		const float mu_vec_1 = mu1;
		const float mu_vec_2 = mu2;

		//Termine F_ξ (per ora trascurato -> 0.0f)
		//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		// Stati del sistema
		// ================================
		// Calcolo F_xi3 e F_xi4 (paper)
		// ================================

		// ψ è l’angolo di yaw del veicolo
		// qui assumiamo che tu abbia già:
		//   cos_yaw = cos(psi)
		//   sin_yaw = sin(psi)
		// e yaw_rate_fb = r = dψ/dt

		const float cpsi = cos_yaw;
		const float spsi = sin_yaw;
		const float r    = yaw_rate_fb;   // yaw rate reale

		// 1) Ricostruzione delle velocità relative u_r e v_r
		//    dalle eq. (9e)-(9f) del paper:
		//
		// [ xi3 + l*r*sin(psi) ]   [ cos(psi)  -sin(psi) ] [ u_r ]
		// [ xi4 - l*r*cos(psi) ] = [ sin(psi)   cos(psi) ] [ v_r ]
		//
		// => [u_r; v_r] = R(psi)^T * [xi3 + l*r*sin(psi); xi4 - l*r*cos(psi)]

		const float beta1 = xi3 + l_pf * r * spsi;
		const float beta2 = xi4 - l_pf * r * cpsi;

		const float u_r =  cpsi * beta1 + spsi * beta2;
		const float v_r = -spsi * beta1 + cpsi * beta2;

		// 2) Parametri dinamici del modello 3-DOF del tuo USV
		//    (parte rigida: M ≈ diag(20, 20, 2.92), D ≈ diag(0.7, 0.7, 2.2))
		//    qui le added-mass sono nulle (xDotU = yDotV = nDotR = 0)

		constexpr float m   = 20.0f;   // m11 = m22
		constexpr float Iz  = 2.92f;   // m33
		constexpr float d11 = 0.7f;    // damping in surge
		constexpr float d22 = 0.7f;    // damping in sway
		constexpr float d33 = 2.2f;    // damping in yaw

		// coefficienti normalizzati (d_ij / m_ij)
		const float d11_over_m   = d11 / m;
		const float d22_over_m   = d22 / m;
		const float d33_over_m33 = d33 / Iz;

		// 3) Termini interni a e b (vedi derivazione dal paper):
		//    F_ur = v_r * r - (d11/m) * u_r
		//    a    = F_ur - v_r * r - l * r^2
		//         = -(d11/m)*u_r - l*r^2
		//
		//    F_r  ≈ -(d33/Iz) * r
		//    b    = u_r*r + X*r + Y*v_r + F_r*l
		//         ≈ -(d22/m)*v_r - (d33/Iz)*l*r   (per il modello semplificato che stiamo usando)

		const float a = -d11_over_m * u_r - l_pf * r * r;
		const float b = -d22_over_m * v_r - d33_over_m33 * l_pf * r;

		// 4) Rotazione (eq. (11) del paper):
		// [F_xi3]   [  cos(psi)  -sin(psi) ] [ a ]
		// [F_xi4] = [  sin(psi)   cos(psi) ] [ b ]

		const float F_xi3 =  cpsi * a - spsi * b;
		const float F_xi4 =  spsi * a + cpsi * b;
		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// const float F_xi3 = 0.0f; // TODO: inserisci qui il modello dinamico completo se vuoi
		// const float F_xi4 = 0.0f;

		const float mu_minus_F_1 = mu_vec_1 - F_xi3;
		const float mu_minus_F_2 = mu_vec_2 - F_xi4;

		//Tau = [τ_u; τ_r] = R_h^{-1} ( μ - F_ξ )
		// const float tau_u = Rh_inv_11 * mu_minus_F_1 + Rh_inv_12 * mu_minus_F_2;
		// const float tau_r = Rh_inv_21 * mu_minus_F_1 + Rh_inv_22 * mu_minus_F_2;
		float tau_u = Rh.I()(0,0) * mu_minus_F_1 + Rh.I()(0,1) * mu_minus_F_2;
		float tau_r = Rh.I()(1,0) * mu_minus_F_1 + Rh.I()(1,1) * mu_minus_F_2;	


		// ----- CONTROL INPUT AI THRUSTER (MATRICE DI ALLOCAZIONE) -----
		// Interpretiamo:
		//  - primo argomento = τ_u (forza in surge)
		//  - secondo argomento = τ_r (momento in yaw)
		// Sarà poi getControlInput a fare:
		//  T_R = 0.5 τ_u + 0.5 τ_r / d
		//  T_L = 0.5 τ_u - 0.5 τ_r / d
		// (o logica equivalente nel tuo mixer PX4).

		// if (fabs(mu1) > fabs(mu2))
		// {
		// 	control_input = getControlInput(2.0f * tau_u, tau_r);
		// }
		// else {
		// 	if (fabs(mu2)-fabs(mu1) < 0.2f)
		// 	{
		// 		control_input = getControlInput(tau_u, 1.2f * tau_r);
		// 	}
		// 	else
		// 	{
		// 		control_input = getControlInput(0.1f * tau_u, 2.0f * tau_r);
		// 	}
		// }
		

		control_input = getControlInput(tau_u, tau_r);

		// Print mu1, mu2, tau_u, tau_r
		PX4_INFO("mu1: %.2f, mu2: %.2f, tau_u: %.2f, tau_r: %.2f", (double)mu1, (double)mu2, (double)tau_u, (double)tau_r);
		// Print control inputs
		PX4_INFO("Control inputs - Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));
		// Print position of hand frame and reference point on path
		PX4_INFO("Hand point xi1: %.2f, xi2: %.2f, Path point x_d: %.2f, y_d: %.2f", (double)xi1, (double)xi2, (double)x_d, (double)y_d);

		// ----- DEBUG -----
		// PX4_INFO("PF-FL: e1=%.2f e2=%.2f e3=%.2f e4=%.2f", (double)e1, (double)e2, (double)e3, (double)e4);
		// PX4_INFO("PF-FL: mu1=%.2f mu2=%.2f tau_u=%.2f tau_r=%.2f", (double)mu1, (double)mu2, (double)tau_u, (double)tau_r);
		// PX4_INFO("PF-FL: s=%.2f/%0.2f U_cmd=%.2f", (double)s_pf, (double)path_len, (double)U_cmd);

		// ----- POPOLA MESSAGGIO DI DIAGNOSTICA -----
		marine_navigation.timestamp          = hrt_absolute_time();
		marine_navigation.desired_speed      = U_cmd;
		marine_navigation.desired_angular_vel= 0.0f;          // non stiamo usando un setpoint esplicito di yaw qui
		marine_navigation.speed_error        = 0.0f;          // non abbiamo PI sulla speed, ma potresti aggiungerlo se vuoi
		marine_navigation.angular_vel_error  = 0.0f;
		marine_navigation.omega_desired_z    = 0.0f;
		marine_navigation.force_input        = 0.0f;
		marine_navigation.torque_input       = 0.0f;
		marine_navigation.angular_error      = 0.0f;

		marine_navigation.q_desired[0] = 0.f;
		marine_navigation.q_desired[1] = 0.f;
		marine_navigation.q_desired[2] = 0.f;
		marine_navigation.q_desired[3] = 0.f;
		marine_navigation.q_feedback[0] = 0.f;
		marine_navigation.q_feedback[1] = 0.f;
		marine_navigation.q_feedback[2] = 0.f;
		marine_navigation.q_feedback[3] = 0.f;
		marine_navigation.q_error[0]    = 0.f;
		marine_navigation.q_error[1]    = 0.f;
		marine_navigation.q_error[2]    = 0.f;
		marine_navigation.q_error[3]    = 0.f;
	}
	// If NOT in manual control mode, stop the servos
	else if(!in_marine_mode) {
		// actuator_servos_s actuator_servos{};
		// actuator_servos.timestamp = hrt_absolute_time();
		// actuator_servos.control[0] = 0; 
		// actuator_servos.control[1] = 0; 
		// _actuator_servos_pub.publish(actuator_servos);

		control_input = Vector2f(0.0f, 0.0f);
		
		//marine_navigation_s marine_navigation{};
		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.q_desired[0] = 0;
		marine_navigation.q_desired[1] = 0;
		marine_navigation.q_desired[2] = 0;
		marine_navigation.q_desired[3] = 0;
		marine_navigation.q_feedback[0] = 0;
		marine_navigation.q_feedback[1] = 0;
		marine_navigation.q_feedback[2] = 0;
		marine_navigation.q_feedback[3] = 0;
		marine_navigation.q_error[0] = 0;
		marine_navigation.q_error[1] = 0;
		marine_navigation.q_error[2] = 0;
		marine_navigation.q_error[3] = 0;
		marine_navigation.desired_speed = 0;
		marine_navigation.desired_angular_vel = 0;
		marine_navigation.speed_error = 0;
		marine_navigation.angular_vel_error = 0;
		marine_navigation.omega_desired_z = 0;
		marine_navigation.force_input = 0;
		marine_navigation.torque_input = 0;
		marine_navigation.angular_error = 0;

		//_marine_navigation_pub.publish(marine_navigation);

		// Reset variables
		if (module_initialization) {
			module_initialization = false; // Reset initialization flag
		}
	}

	// Publish on actuator_servos topic even if not armed
	actuator_servos_s actuator_servos{};
	actuator_servos.timestamp = hrt_absolute_time();
	actuator_servos.control[0] = control_input(0); // Left propeller control input
	actuator_servos.control[1] = control_input(1); // Right propeller
	_actuator_servos_pub.publish(actuator_servos);	

	_marine_navigation_pub.publish(marine_navigation);

	perf_end(_loop_perf);
}

Vector3f MarineNavigation::getRPY(const Quatf &q)
{
	Eulerf euler(q);
	return Vector3f(euler(0), euler(1), euler(2));
}

void MarineNavigation::updateQDesired(const float &d_t, const float &omega_z)
{
	Quatf quat_d_n1 = Quatf(quat_d(0)*cos(omega_z * d_t / 2) + quat_d(3) * sin(omega_z * d_t / 2) - d_t * leak * (quat_d(0) - quat_fb(0)), 
	0, 
	0, 
	- quat_d(0)*sin(omega_z * d_t / 2) + quat_d(3) * cos(omega_z * d_t / 2) - d_t * leak * (quat_d(3) - quat_fb(3))); // Update desired quaternion considering leakage
	quat_d_n1.normalize();
	quat_d = quat_d_n1;
}

Vector2f MarineNavigation::getControlInput(const float &force_input, const float &T_input)
{

	SquareMatrix<float, 2> allocation_matrix;
	allocation_matrix(0, 0) = Prop_C;
	allocation_matrix(0, 1) = Prop_C;
	allocation_matrix(1, 0) = -left_th_y * Prop_C; // Right thruster x position
	allocation_matrix(1, 1) = -right_th_y * Prop_C; // Right thruster y position

	Vector2f computed_input;
	// Calculate the control input for each thruster based on the force and yaw speed inputs
	// computed_input(0) = allocation_matrix.I()(0, 0) * force_input * max_propeller_speed + allocation_matrix.I()(0, 1) * -T_input; // Left thruster input
	// computed_input(1) = allocation_matrix.I()(1, 0) * force_input * max_propeller_speed + allocation_matrix.I()(1, 1) * -T_input; // Right thruster input

	computed_input(0) = allocation_matrix.I()(0, 0) * force_input + allocation_matrix.I()(0, 1) * -T_input; // Left thruster input
	computed_input(1) = allocation_matrix.I()(1, 0) * force_input + allocation_matrix.I()(1, 1) * -T_input; // Right thruster input

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
	if (fabs(omega_input) <= 0.009f) {
		omega_filtered = 0.0f;	
	}
	else {
		omega_filtered = omega_input; 
	}

	omega_filtered = omega_filtered * max_yawspeed;

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