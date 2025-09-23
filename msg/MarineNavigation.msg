uint64 timestamp		# time since system start (microseconds)

float32[4] q_desired	# Desired orientation in quaternion
float32[4] q_feedback	# Feedback orientation in quaternion (ref frame with z-axis always alligned with the world z-axis)
float32[4] q_error		# Error orientation in quaternion

float32 desired_thrust
float32 desired_angular_vel
float32 angular_vel_error
float32 omega_desired_z
float32 torque_input
float32 angular_error
