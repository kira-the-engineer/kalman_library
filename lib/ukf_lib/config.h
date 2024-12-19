/*
 * Configuration file for working with the embedded UKF class. Defines macros that allow for compile time sizing of matrices used in the class
 */

#ifndef CONFIG_H
#define CONFIG_H

/*Set Dimensions*/
#define STATE_DIM 2 //(int like) this macro controls how many states are estimated by the filter. For example if we wanted to use the 
                      //UKF to estimate position and velocity in 1D- we'd have 2 states
#define MEASUREMENT_DIM 1 //(int like) this macro controls how many measurements can be expected by the filter. For example a sensor 
                            //providing position in (x,y) would have a dimension of 2.

/*
 * Sigma point generation parameters.
 * These are specific to the sigma point generation method outlined in Van Der Merwe's dissertation.
 * Since the sigma points are chosen to capture the 1st two moments (mean and variance) of the distribution of the
 * state, the tuning parameters defined here determine how closely the sigma points match the moments
 */

#define ALPHA 0.3 //(float like). Changing this parameter determines how far/close the sigma points are to the mean
#define BETA 2.0 //(float like). This parameter is used to incorporate prior knowledge about the distribution of the state. Helps minimize errors in the covariance estimate. 
#define KAPPA 0.1 //(float like). Additional scaling parameter that determines how far the points are spread around the mean

#define TIME_STEP 0.1 //(float like) time between each step in seconds (dt)

#endif