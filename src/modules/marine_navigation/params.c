/**
 * @file params.c
 *
 * Parameters defined by the marine navigation module
 *
 * @author Andrea Capuozzo <andrea.capuozzo@unina.it>
 */

/**
 * Propeller constant
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_PROP_C, 1.0f);

/**
 * Gain for omega desired computation
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_KQ, 4.6f);

/**
 * Proportional gain for torque computation
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_KR, 2.4f);

/**
 * Leak factor for increment of desired quaternion
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_LEAK, 1.1f);

/**
 * Maximum linear speed
 *
 * @unit N
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_MAX_SP, 1.0f);

/**
 * Maximum yaw speed
 *
 * @unit rad/s
 * @min 0.0
 * @max 3.14
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_MAX_YAW, 0.24f);   

/**
 * Left thruster X position
 *
 * @unit m
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_L_X, 0.0f);   

/**
 * Left thruster Y position
 *
 * @unit m
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_L_Y, 0.0f); 

/**
 * Right thruster X position
 *
 * @unit m
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_R_X, 0.0f);

/**
 * Right thruster Y position
 *
 * @unit m
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_R_Y, 0.0f);



