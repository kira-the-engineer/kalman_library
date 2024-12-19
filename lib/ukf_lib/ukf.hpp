/*
 * Definition file for the Unscented Kalman Filter class. End users should not be touching this- if you're wanting to configure certain parameters that affect how the class works, please
 * use "config.h"!
*/

#include "config.h"
#include "eigen.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>

using namespace Eigen;
using namespace std;

class UKF {
    public:
        UKF(); //empty constructor
        /*
         * Constructor for a basic unscented kalman filter object.
         * Parameters:
         * State: Vector of the initial values of the state of the system
         * Cov: Matrix that holds the initial covariance estimate of the system
         * Proc_noise: Matrix that holds the noise in the system (Q)
         * Meas_noise: Matrix that holds the noise in the measurement (R)
         * process (function): Function that moves the sigma points forward in time according to the process model. Should return a matrix of transformed 
         * sigma points. Takes in the sigma points matrix and the time step used to forward each point
         * meas (function): Function responsible for converting the prior sigma points returned by f_x into measurements. Should return a matrix
         */
        UKF(VectorXf state, MatrixXf cov, MatrixXf proc_noise, MatrixXf meas_noise, MatrixXf (*process)(MatrixXf, float), MatrixXf (*meas)(MatrixXf));

        ~UKF(); //deconstructor

        /*
         * Function that allows user to set internal function pointers to functions for working with nonlinear states/measurements. For example, many localization problems use angular states
         * which need to be normalized to ensure validity. If NULL pointers are passed- no change will happen and the default subtract, add, and mean functions are used. 
         */
        void init_nonlinear(VectorXf (*add)(VectorXf, VectorXf), VectorXf (*sub)(VectorXf, VectorXf), VectorXf (*ux)(MatrixXf, RowVectorXf), VectorXf (*uz)(MatrixXf, RowVectorXf));

        /*
         * Predict
         */
        void predict();

        /*
         * Update
         */
        void update(VectorXf z);

        /*
         * Computes the sigma points for the UKF using Van Der Merwe's method
         * Parameters:
         * mean (vector): n-length vector that holds the mean/state of the filter
         * cov (matrix): Covariance of filter
         * Returns:
         * sigmas: Matrix of size (2*dim_x+1, n). Each col contains sigma points for 1 dimension of the problem space
         */
        MatrixXf generate_sigmas(VectorXf mean, MatrixXf cov);

        /*
         * Performs the unscented transform according to Van der Merwe's Paper- called in both update and predict fxns
         * Used to calculate the mean and covariance of the sigma points
         * Parameters:
         * mean: used to update pointer to the mean vector
         * cov: used to update pointer to the covariance vector
         * sigmas: Matrix of sigma points to pass through the transformation functions (fx/hx)
         * noise: Noise matrix to add to the final calculated covariance. If the system/measurements are noiseless (not likely), this would be a zero matrix.
         * nl_mean: (optional): a function that deals with nonlinear values in the state/measurement when calculating the mean
         * nl_sub (optional): a function that supports nonlinearity in getting the difference between the sigma points and the mean vector
         */
        void unscented_transform(Ref<VectorXf> mean, Ref<MatrixXf> cov, MatrixXf sigmas, RowVectorXf wm, RowVectorXf wc, MatrixXf noise, VectorXf (*nl_mean)(MatrixXf, RowVectorXf) = NULL, VectorXf (*nl_sub)(VectorXf, VectorXf) = NULL);
    
    private:
        /*sigma parameters*/
        float lambda;

        /*Vectors and Matrices*/
        
        //Vectors
        //state: Stores the estimate of the state (mean). Updated whenever the predict-update sequence is called. Denoted as "x" in literature
        //data: Stores data from sensors/signals. Denoted as "z" in literature
        //residual: vector that's updated whenever update is called. The error between the data points and their calculated mean
        Vector<float, STATE_DIM> state;
        Vector<float, MEASUREMENT_DIM> data;
        Vector<float, MEASUREMENT_DIM> residual;

        //Weight vectors
        RowVector<float, 2*STATE_DIM+1> w_mean;
        RowVector<float, 2*STATE_DIM+1> w_cov;


        //Matricies
        //state_cov: Covariance matrix for the state estimate <dim_s rows, dim_s cols>. Any call to the update or predict updates this variable (P in literature)
        //meas_cov: Also known as the system uncertainty. Covariance of the sigma points transformed by h_x. <dim_m rows, dim_m cols> (S in literature)
        //meas_covI: inverse of measurement covariance, used in calculating the kalman gain (SI in literature)
        //meas_err: Error/noise in data/measurement. <dim_m rows, dim_m cols>
        //proc_err: Error/noise in the process model. <dim_s rows, dim_s cols>
        //cross_cov: Matrix to store the cross covariance between the state and the data/measurements <dim_s rows, dim_m cols>
        //gain: Kalman gain. In the case of the UKF we can think of gain as a ratio of our belief in the state estimate vs our belief in the measurement
        //sigmas_s: Matrix that stores the sigma points transformed by f_x (the process sigmas).  <num_sigmas rows, dim_s cols>
        //sigmas_m: Matrix that stores the sigma points transformed by h_x (the measurment sigmas). <num_sigmas rows, dim_m cols>
        Matrix<float, STATE_DIM, STATE_DIM> state_cov;
        Matrix<float, MEASUREMENT_DIM, MEASUREMENT_DIM> meas_cov;
        Matrix<float, MEASUREMENT_DIM, MEASUREMENT_DIM> meas_covI;
        Matrix<float, MEASUREMENT_DIM, MEASUREMENT_DIM> meas_err;
        Matrix<float, STATE_DIM, STATE_DIM> proc_err;
        Matrix<float, STATE_DIM, MEASUREMENT_DIM> cross_cov;
        Matrix<float, STATE_DIM, MEASUREMENT_DIM> gain;
        Matrix<float, 2 * STATE_DIM + 1, STATE_DIM> sigma_s;
        Matrix<float, 2 * STATE_DIM + 1, MEASUREMENT_DIM> sigmas_m;



        //function pointers
        MatrixXf (*f_x)(MatrixXf, float) = NULL; //transformation function to forward sigma points according to process model
        MatrixXf (*h_x)(MatrixXf) = NULL; //transformation function to forward sigma points according to measurement model

        //nonlinear functions
        VectorXf (*nl_sub)(VectorXf, VectorXf) = NULL; //Used for calculating the residual when there are nonlinear values that need to be normalized (like angles)
        VectorXf (*nl_add)(VectorXf, VectorXf) = NULL; //used to add vectors with nonlinear values when updating the state estimate
        VectorXf (*state_mean)(MatrixXf sigmas_s, RowVectorXf W_mean) = NULL; //used to take the mean of non-linear state variables (for example, averaging angles usually involves using atan2)
        VectorXf (*meas_mean)(MatrixXf sigmas_h, RowVectorXf W_mean) = NULL; //same as above, but called when there are non-linear variables in the measurement (this is so the two can be called independantly)


        void set_weights(); // Calculates weights according to Van Der Merwe's paper- should only be called in constructor

        
};