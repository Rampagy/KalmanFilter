/*!
* \brief Test of the Kalman filter
*
* In this example, an accelerometer vector (x-axis) will be estimated using both a
* gyro and accelerometer. The gyro has a variance of var(s) = 0.6m/s/s and the 
* accelerometer has a variance of car(s) = 0.4m/s/s.
*
* The time constant is set to T = 1s.
*
* The initial estimation of the vector is set to 0m/s/s.
*/

#define EXTERN_INLINE_MATRIX static INLINE
#define EXTERN_INLINE_KALMAN static INLINE

#include <assert.h>
#include <stdio.h>
#include "kalman_test.h"

// create the filter structure
#define KALMAN_NAME acceleration_x
#define KALMAN_NUM_STATES 2
#define KALMAN_NUM_INPUTS 0
#include "kalman_factory_filter.h"

// create the measurement structure
#define KALMAN_MEASUREMENT_NAME acceleration_x_axis
#define KALMAN_NUM_MEASUREMENTS 2
#include "kalman_factory_measurement.h"

// clean up
#include "kalman_factory_cleanup.h"

/*!
* \brief Initializes the gravity Kalman filter
*/
static void kalman_gravity_init()
{
    /************************************************************************/
    /* initialize the filter structures                                     */
    /************************************************************************/
    kalman_t *kf = kalman_filter_gravity_init();
    kalman_measurement_t *kfm = kalman_filter_gravity_measurement_position_init();

    /************************************************************************/
    /* set initial state                                                    */
    /************************************************************************/
    matrix_t *x = kalman_get_state_vector(kf);
    x->data[0] = 0; // a_i (accelerometer init)
    x->data[1] = 0; // g_i (gyro init)

    /************************************************************************/
    /* set state transition                                                 */
    /************************************************************************/
    matrix_t *A = kalman_get_state_transition(kf);

    // set time constant
    const matrix_data_t T = 1;

    // transition of x to s
    matrix_set(A, 0, 0, 1);   // 1
    matrix_set(A, 0, 1, T);   // T
    matrix_set(A, 0, 2, (matrix_data_t)0.5*T*T); // 0.5 * T^2

    // transition of x to v
    matrix_set(A, 1, 0, 0);   // 0
    matrix_set(A, 1, 1, 1);   // 1
    matrix_set(A, 1, 2, T);   // T

    // transition of x to g
    matrix_set(A, 2, 0, 0);   // 0
    matrix_set(A, 2, 1, 0);   // 0
    matrix_set(A, 2, 2, 1);   // 1

    /************************************************************************/
    /* set covariance                                                       */
    /************************************************************************/
    matrix_t *P = kalman_get_system_covariance(kf);

    matrix_set_symmetric(P, 0, 0, (matrix_data_t)0.1);   // var(s)
    matrix_set_symmetric(P, 0, 1, 0);   // cov(s,v)
    matrix_set_symmetric(P, 0, 2, 0);   // cov(s,g)

    matrix_set_symmetric(P, 1, 1, 1);   // var(v)
    matrix_set_symmetric(P, 1, 2, 0);   // cov(v,g)

    matrix_set_symmetric(P, 2, 2, 1);   // var(g)

    /************************************************************************/
    /* set measurement transformation                                       */
    /************************************************************************/
    matrix_t *H = kalman_get_measurement_transformation(kfm);

    matrix_set(H, 0, 0, 1);     // z = 1*s
    matrix_set(H, 0, 1, 0);     //   + 0*v
    matrix_set(H, 0, 2, 0);     //   + 0*g

    /************************************************************************/
    /* set process noise                                                    */
    /************************************************************************/
    matrix_t *R = kalman_get_process_noise(kfm);

    matrix_set(R, 0, 0, (matrix_data_t)0.5);     // var(s)
}

// define measurements.
//
// MATLAB source
// -------------
// s = s + v*T + g*0.5*T^2;
// v = v + g*T;
#define MEAS_COUNT (15)
static matrix_data_t real_distance[MEAS_COUNT] = {
    (matrix_data_t)0,
    (matrix_data_t)4.905,
    (matrix_data_t)19.62,
    (matrix_data_t)44.145,
    (matrix_data_t)78.48,
    (matrix_data_t)122.63,
    (matrix_data_t)176.58,
    (matrix_data_t)240.35,
    (matrix_data_t)313.92,
    (matrix_data_t)397.31,
    (matrix_data_t)490.5,
    (matrix_data_t)593.51,
    (matrix_data_t)706.32,
    (matrix_data_t)828.94,
    (matrix_data_t)961.38 };

// define measurement noise with variance 0.5
//
// MATLAB source
// -------------
// noise = 0.5^2*randn(15,1);
static matrix_data_t measurement_error[MEAS_COUNT] = {
    (matrix_data_t)0.13442,
    (matrix_data_t)0.45847,
    (matrix_data_t)-0.56471,
    (matrix_data_t)0.21554,
    (matrix_data_t)0.079691,
    (matrix_data_t)-0.32692,
    (matrix_data_t)-0.1084,
    (matrix_data_t)0.085656,
    (matrix_data_t)0.8946,
    (matrix_data_t)0.69236,
    (matrix_data_t)-0.33747,
    (matrix_data_t)0.75873,
    (matrix_data_t)0.18135,
    (matrix_data_t)-0.015764,
    (matrix_data_t)0.17869 };

/*!
* \brief Runs the gravity Kalman filter.
*/
void kalman_gravity_demo()
{
    // initialize the filter
    kalman_gravity_init();

    // fetch structures
    kalman_t *kf = &kalman_filter_gravity;
    kalman_measurement_t *kfm = &kalman_filter_gravity_measurement_position;

    matrix_t *x = kalman_get_state_vector(kf);
    matrix_t *z = kalman_get_measurement_vector(kfm);

    // filter!
    for (int i = 0; i < MEAS_COUNT; ++i)
    {
        // prediction.
        kalman_predict(kf);

        // measure ...
        matrix_data_t measurement = real_distance[i] + measurement_error[i];
        matrix_set(z, 0, 0, measurement);

        // update
        kalman_correct(kf, kfm);
    }

    // fetch estimated g
    matrix_data_t g_estimated = x->data[2];
    assert(g_estimated > 9 && g_estimated < 10);
    printf("kalman: %f\n", g_estimated);
}