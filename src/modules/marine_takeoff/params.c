/**
 * @file params.c
 *
 * Parameters defined by the marine takeoff module
 *
 * @author Paola Percuoco
 */


/**
 * Attitude threshold (radians)
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Marine Takeoff
 */
PARAM_DEFINE_FLOAT(MARINE_ATT_TH, 0.069f);

/**
 * Minimum window time for takeoff (seconds)
 *
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Marine Takeoff
 */
PARAM_DEFINE_FLOAT(MARINE_WIN_TIME, 0.8f);
