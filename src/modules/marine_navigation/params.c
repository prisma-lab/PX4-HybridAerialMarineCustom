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
 * Proportional gain for thrust computation
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation 
 */
PARAM_DEFINE_FLOAT(MARINE_KT, 10.0f);

/**
 * Proportional gain for thrust computation in feedforward
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation
 */
PARAM_DEFINE_FLOAT(MARINE_KT_FF, 10.0f);

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
 * Quadratic drag coefficient
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Marine Navigation
 */
PARAM_DEFINE_FLOAT(MARINE_DRAG, 1.1f);

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

/**
 * L parameter for hand-position point
 *
 * @unit m
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_LPF, 1.5f);


/**
 *Parameter to comput virtual input for autonomous marine mode
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_PX, 0.7f);

/**
 *Parameter to comput virtual input for autonomous marine mode
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_PY, 0.7f);

/**
 *Parameter to comput virtual input for autonomous marine mode
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_VX, 1.0f);

/**
 *Parameter to comput virtual input for autonomous marine mode
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_VY, 1.0f);

/**
 *Parameter to comput virtual input for autonomous marine mode
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_IX, 0.05f);

/**
 *Parameter to comput virtual input for autonomous marine mode
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_IY, 0.05f);

/**
 *Integration leakage
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_IL, 0.15f);

/**
 *Integral saturation limit
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_I_MAX, 1.5f);

/**
 *Gain for current estimation
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_C, 0.25f);

/**
 *Leakage for current estimation
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_K_CL, 0.02f);

/**
 *Max velocity for estimated sea current (used for saturation in the observer)
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_VC_MAX, 2.0f);

/**
 *Minimum velocity for adaptation of sea current estimation
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_VMIN_ADAPT, 0.5f);

/**
 *Desired cruising speed
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_V_CRUISE, 6.5f);

/**
 *Distance from target waypoint from which the vessel starts to slow down
 *
 * @min 0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Marine Auto Mission 
 */
PARAM_DEFINE_FLOAT(AUTO_D_SLOW, 2.0f);



