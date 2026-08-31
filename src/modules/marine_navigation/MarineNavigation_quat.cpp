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
	ScheduleOnInterval(20_ms); // 50Hz
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
		// Parameters for auto mission
		l_pf = _param_auto_lpf.get();
		k_px = _param_auto_k_px.get();
		k_py = _param_auto_k_py.get();
		k_vx = _param_auto_k_vx.get();
		k_vy = _param_auto_k_vy.get();
		k_Ix = _param_auto_k_Ix.get();
		k_Iy = _param_auto_k_Iy.get();
		k_il = _param_auto_k_il.get();
		I_max = _param_auto_i_max.get();
		k_c = _param_auto_k_c.get();
		k_cl = _param_auto_k_cl.get();
		vc_max = _param_auto_vc_max.get();
		vmin_adapt = _param_auto_vmin_adapt.get();
		v_cruise_auto = _param_v_cruise_auto.get();
		d_slow_auto = _param_d_slow_auto.get();
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

	bool new_pos_sp_triplet = false;
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
		new_pos_sp_triplet = true;
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
				quat_fb = Quatf(cos(yaw_cont / 2), 0, 0, sin(yaw_cont / 2)); // Quaternion feedback from yaw angle
			}

			else if(!module_initialization) {
				yaw_cont = rpy(2);
				yaw_fb_prev = rpy(2);
				quat_fb = Quatf(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2)); // Quaternion feedback from yaw angle
				quat_d = quat_fb; // Initialize desired quaternion with the first feedback
				module_initialization = true; // Set flag to true after initialization
			}
		}

		if (_vehicle_angular_velocity_sub.updated()) {
   			_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
	  		yaw_rate_fb = vehicle_angular_velocity.xyz[2]; 
 		}
		
		// Update quaternion desired based on angular velocity input
		updateQDesired(dt, computeOmegaInput(-1.0f * rc_input.roll)); 
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
		torque_input = K_r * (omega_d(2) - yaw_rate_fb);

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

		// Publish on marine_navigation topic
		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.mode = 1.0f;
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

		// UNCOMMENT FOR  LIVE INFO AND DEBUGGING
		// PX4_INFO("Y fb: %.2f, YR fb: %.2f, Vx fb: %.2f", (double)rpy(2), (double)yaw_rate_fb, (double)v_x);
		// PX4_INFO("Vx input: %.2f, Omega z input: %.2f", (double)(rc_input.throttle*max_propeller_speed), (double)computeOmegaInput(rc_input.roll));
		// PX4_INFO("Vx error: %.2f, Omega z error: %.2f", (double)(rc_input.throttle*max_propeller_speed - v_x), (double)(computeOmegaInput(rc_input.roll) - yaw_rate_fb));
		// PX4_INFO("quat_d [0]: %.2f, quat_fb [0]: %.2f", (double)quat_d(0), (double)quat_fb(0));
		// PX4_INFO("quat_d [3]: %.2f, quat_fb [3]: %.2f", (double)quat_d(3), (double)quat_fb(3));
		// PX4_INFO("Quat error [0]: %.2f, Quat error [3]: %.2f, YR error : %.2f", (double)quat_error(0), (double)quat_error(3), (double)(omega_d(2) - yaw_rate_fb));
		// PX4_INFO("Omega d: %.2f, %.2f, %.2f", (double)omega_d(0), (double)omega_d(1), (double)omega_d(2));
		// PX4_INFO("angular error: %.2f", (double)(2.0f * acosf(fabsf(quat_error(0)))));
		// PX4_INFO("Throttle: %.2f, Roll input : %.2f, Force input: %.2f, Torque Input: %.2f", (double)rc_input.throttle, (double)rc_input.roll, (double)force_input, (double)torque_input);
		// PX4_INFO("Control Input Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));
	}
	else if (vehicle_control_mode.flag_control_prisma_marine_manual_ts_enabled) {

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
			control_input(1) = -(thrust +1)*steering +thrust;
		} else {
			control_input(0) = (thrust +1)*steering +thrust;
			control_input(1) = thrust;
		}

		// UNCOMMENT FOR  LIVE INFO AND DEBUGGING
		// // Print joystick input and controls
		// PX4_INFO("Joystick Input - Throttle: %.2f, Roll: %.2f", (double)rc_input.throttle, (double)rc_input.roll);
		// PX4_INFO("Control Input - Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));

		//marine_navigation_s marine_navigation{};
		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.mode = 2.0f;
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

		// Reset variables
		if (module_initialization) {
			module_initialization = false; // Reset initialization flag
		}
	}
	else if (vehicle_control_mode.flag_control_prisma_marine_manual_ff_enabled) {

		if(fabs(rc_input.throttle) < 0.09f) control_input(0) = 0.0f;
		if(fabs(rc_input.pitch) < 0.09f) control_input(1) = 0.0f;
		else control_input = Vector2f(rc_input.throttle, rc_input.pitch);
		
		// UNCOMMENT FOR  LIVE INFO AND DEBUGGING
		// PX4_INFO("Control Input - Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));

		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.mode = 3.0f;
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

		// Local position initialization and validity check
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

		if(new_pos_sp_triplet) {
			// Reset each new waypoint
			s_pf    = 1.0f;
			xi1_I   = 0.0f;
			xi2_I   = 0.0f;
			vc_hat_x = 0.0f;
			vc_hat_y = 0.0f;
			new_pos_sp_triplet = false;
		}

		// Waypoint in local frame
		const float yaw = rpy(2);
		yaw_rate_fb = vehicle_angular_velocity.xyz[2];

		Vector2f curr_pos_local{vehicle_local_position.x, vehicle_local_position.y};

		Vector2f curr_wp_local = _global_local_proj_ref.project(_pos_sp_triplet.current.lat,
									_pos_sp_triplet.current.lon);

		Vector2f prev_wp_local = curr_wp_local;
		if (_pos_sp_triplet.previous.valid) {
			prev_wp_local = _global_local_proj_ref.project(_pos_sp_triplet.previous.lat,
									_pos_sp_triplet.previous.lon);
		}

		// Path tangent vector
		Vector2f path_vec = curr_wp_local - prev_wp_local;
		const float path_len = path_vec.norm();

		if (path_len < 1e-3f) {
			PX4_WARN("Auto marine PF-FL: path segment too short");
			control_input = Vector2f(0.0f, 0.0f);
			return;
		}

		const Vector2f t_hat = path_vec / path_len; // tangente unitaria

		// Hand-point (xi1, xi2)
		const float cos_yaw = cosf(yaw);
		const float sin_yaw = sinf(yaw);

		const float x  = curr_pos_local(0);
		const float y  = curr_pos_local(1);

		const float xi1 = x + l_pf * cos_yaw;
		const float xi2 = y + l_pf * sin_yaw;

		// Reference point on path and error with reference hand-point
		const float x_gamma = prev_wp_local(0) + t_hat(0) * s_pf;
		const float y_gamma = prev_wp_local(1) + t_hat(1) * s_pf;

		const float tilde_xi1 = xi1 - x_gamma;
		const float tilde_xi2 = xi2 - y_gamma;
		const float e_h_sq    = tilde_xi1 * tilde_xi1 + tilde_xi2 * tilde_xi2;

		// Adjust velocity near target
		double current_lat{};
		double current_lon{};
		_global_local_proj_ref.reproject(curr_pos_local(0), curr_pos_local(1), current_lat, current_lon);

		const float dist_target = get_distance_to_next_waypoint(current_lat, current_lon,
									_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

		float U_cmd = v_cruise_auto;
		if (dist_target < d_slow_auto) {
			U_cmd = v_cruise_auto * (dist_target / d_slow_auto);
			U_cmd = math::max(U_cmd, 0.3f);
		}

		// Arclength update
		const float s_dot = U_cmd * (1.0f - tanhf(e_h_sq));

		s_pf += s_dot * dt;
		s_pf = math::constrain(s_pf, 0.0f, path_len);

		const float x_d = prev_wp_local(0) + t_hat(0) * s_pf;
		const float y_d = prev_wp_local(1) + t_hat(1) * s_pf;

		const float x_s = t_hat(0);
		const float y_s = t_hat(1);

		const float x_dot_d = s_dot * x_s;
		const float y_dot_d = s_dot * y_s;

		// TO DO: for curved paths
		//const float x_ddot_star = 0.0f;
		//const float y_ddot_star = 0.0f;

		// Hand-point velocity considering ocean current
		const float x_dot_g = vehicle_local_position.vx;
		const float y_dot_g = vehicle_local_position.vy;

		const float x_dot_rel = x_dot_g - vc_hat_x;
		const float y_dot_rel = y_dot_g - vc_hat_y;

		const float xi1_dot = x_dot_rel - l_pf * sin_yaw * yaw_rate_fb;
		const float xi2_dot = y_dot_rel + l_pf * cos_yaw * yaw_rate_fb;

		const float xi3 = xi1_dot;
		const float xi4 = xi2_dot;

		// Error quantities
		const float e1 = xi1 - x_d;
		const float e2 = xi2 - y_d;
		const float e3 = xi3 - x_dot_d;
		const float e4 = xi4 - y_dot_d;

		// Ocean current estimation
		const float v_surge = vehicle_local_position.vx * cos_yaw + vehicle_local_position.vy * sin_yaw;
		float aligned_gate = 1.0f; 

		Vector2f v_g(vehicle_local_position.vx, vehicle_local_position.vy);
		const float v_norm = v_g.norm();

		if (v_norm > vmin_adapt) {

			const Vector2f v_g_hat = v_g / v_norm;

			const float cos_align = v_g_hat.dot(t_hat);

			aligned_gate = math::max(0.0f, cos_align);

			aligned_gate *= aligned_gate;
		}

		if (aligned_gate > 0.75f && fabsf(v_surge) > vmin_adapt) {

			const float vc_hat_x_dot = k_c * e3 - k_cl * vc_hat_x;
			const float vc_hat_y_dot = k_c * e4 - k_cl * vc_hat_y;

			vc_hat_x += vc_hat_x_dot * dt;
			vc_hat_y += vc_hat_y_dot * dt;

			vc_hat_x = math::constrain(vc_hat_x, -vc_max, vc_max);
			vc_hat_y = math::constrain(vc_hat_y, -vc_max, vc_max);
		}

		// Integral actioion
		const bool integrate_ok = (aligned_gate > 0.75f) && (s_dot > 0.02f) && (fabsf(v_surge) > 0.1f);

		if (integrate_ok) {
			xi1_I += (e1 - k_il * xi1_I) * dt;
			xi2_I += (e2 - k_il * xi2_I) * dt;
		} else {
			xi1_I += (-k_il * xi1_I) * dt;
			xi2_I += (-k_il * xi2_I) * dt;
		}

		xi1_I = math::constrain(xi1_I, -I_max, I_max);
		xi2_I = math::constrain(xi2_I, -I_max, I_max);

		// Virtual intputs computation
		const float mu1 = -k_vx * e3 - k_px * e1 - k_Ix * xi1_I; // + x_ddot_star;
		const float mu2 = -k_vy * e4 - k_py * e2 - k_Iy * xi2_I; // + y_ddot_star;

		// Feedback linearization 
		SquareMatrix<float, 2> Rh;
		Rh(0,0) =  cos_yaw;           Rh(0,1) = -l_pf * sin_yaw;
		Rh(1,0) =  sin_yaw;           Rh(1,1) =  l_pf * cos_yaw;

		// F_xi3, F_xi4 (simplyfied dynamic model)
		const float cpsi = cos_yaw;
		const float spsi = sin_yaw;
		const float r    = yaw_rate_fb;

		const float beta1 = xi3 + l_pf * r * spsi;
		const float beta2 = xi4 - l_pf * r * cpsi;

		const float u_r =  cpsi * beta1 + spsi * beta2;
		const float v_r = -spsi * beta1 + cpsi * beta2;

		constexpr float m   = 20.0f;
		constexpr float Iz  = 2.92f;
		constexpr float d11 = 1.5f;
		constexpr float d22 = 8.0f;
		constexpr float d33 = 2.2f;

		const float d11_over_m   = d11 / m;
		const float d22_over_m   = d22 / m;
		const float d33_over_m33 = d33 / Iz;

		const float a = -d11_over_m * u_r - l_pf * r * r;
		const float b = -d22_over_m * v_r - d33_over_m33 * l_pf * r;

		const float F_xi3 =  cpsi * a - spsi * b;
		const float F_xi4 =  spsi * a + cpsi * b;

		const float mu_minus_F_1 = mu1 - F_xi3;
		const float mu_minus_F_2 = mu2 - F_xi4;

		// Thrust and Torque computation
		const SquareMatrix<float, 2> Rh_inv = Rh.I();

		const float tau_u = Rh_inv(0,0) * mu_minus_F_1 + Rh_inv(0,1) * mu_minus_F_2;
		const float tau_r = Rh_inv(1,0) * mu_minus_F_1 + Rh_inv(1,1) * mu_minus_F_2;

		control_input = getControlInput(tau_u, tau_r);

		// UNCOMMENT FOR  LIVE INFO AND DEBUGGING
		// // Print mu1, mu2, tau_u, tau_r
		// PX4_INFO("mu1: %.2f, mu2: %.2f, tau_u: %.2f, tau_r: %.2f", (double)mu1, (double)mu2, (double)tau_u, (double)tau_r);
		// // Print control inputs
		// PX4_INFO("Control inputs - Left: %.2f, Right: %.2f", (double)control_input(0), (double)control_input(1));
		// // Print position of hand frame and reference point on path
		// PX4_INFO("Hand point xi1: %.2f, xi2: %.2f, Path point x_d: %.2f, y_d: %.2f", (double)xi1, (double)xi2, (double)x_d, (double)y_d);

		// Use message for debug
		marine_navigation.timestamp          = hrt_absolute_time();
		marine_navigation.mode               = 4.0f;
		marine_navigation.desired_speed      = U_cmd;
		marine_navigation.desired_angular_vel= 0.0f;          
		marine_navigation.speed_error        = 0.0f;          
		marine_navigation.angular_vel_error  = 0.0f;
		marine_navigation.omega_desired_z    = 0.0f;
		marine_navigation.force_input        = tau_u;
		marine_navigation.torque_input       = tau_r;
		marine_navigation.angular_error      = 0.0f;

		marine_navigation.q_desired[0] = curr_wp_local(0); 
		marine_navigation.q_desired[1] = curr_wp_local(1); 
		marine_navigation.q_desired[2] = x_d;
		marine_navigation.q_desired[3] = y_d;
		marine_navigation.q_feedback[0] = curr_pos_local(0);
		marine_navigation.q_feedback[1] = curr_pos_local(1);
		marine_navigation.q_feedback[2] = 0.f;
		marine_navigation.q_feedback[3] = 0.f;
		marine_navigation.q_error[0]    = 0.f;
		marine_navigation.q_error[1]    = 0.f;
		marine_navigation.q_error[2]    = 0.f;
		marine_navigation.q_error[3]    = 0.f;
	}
	// If NOT in manual control mode, stop the servos
	else if(!in_marine_mode) {

		control_input = Vector2f(0.0f, 0.0f);
		
		marine_navigation.timestamp = hrt_absolute_time();
		marine_navigation.mode = 0.0f;
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
	if (fabs(omega_input) <= 0.02f) {
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