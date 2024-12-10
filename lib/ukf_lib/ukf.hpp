// for use when building on MCU
// #include "eigen.h"
// #include <Eigen/Cholesky>
// #include <Eigen/LU>

#include "src_eigen/eigen.h"
#include "math.h"
#include "src_eigen/Eigen/Cholesky"
#include "src_eigen/Eigen/LU"

using namespace Eigen;
using namespace std;

class UKF {
    public:
        UKF(); //empty constructor
        /*
         * Constructor for a basic unscented kalman filter object.
         * Parameters:
         * dim_s (int): dimension of the state vector- pertains to the number of state variables used in the filter (i.e how many rows in the vector)
         * dim_m (int): dimension of the measurement vector- changes based on if you have multiple sensors and what each sensor outputs. For instance
         * if you have a single position sensor with outputs (x,y) z would be a column vector with 2 rows. 
         * dim_c (int): dimension of the control vector, where number of rows is the number of control inputs. defaults to 1 by 1 vector
         * dt (float): time step used in the state transition process- in seconds
         * process (function): Function that moves the sigma points forward in time according to the process model. Should return a matrix of transformed 
         * sigma points. Takes in the sigma points matrix and the time step used to forward each point
         * meas (function): Function responsible for converting the prior sigma points returned by f_x into measurements. Should return a matrix
         * alpha (float): determines spread of sigma points around the mean when generating points (small positive value btwn 0 and 1)
         * beta (float): incorporates prior knowledge of the distribution of the mean
         * kappa (float): Secondary scaling parameter- most often set to zero according to Van der Merwe's paper but sometimes set to 3-n for gaussians
         */ 
        UKF(int dim_s, int dim_m, float time, float alpha, float beta, MatrixXf (*process)(MatrixXf, float), 
            MatrixXf (*meas)(MatrixXf), float kappa, int dim_c);

        ~UKF(); //deconstructor


        /*
         * Init
         * Function that allows the end user to specify initial values for the state vector, process covariance, process noise, and measurement noise matricies. Should only be called once.
         */
        void init(VectorXf state, MatrixXf proc_cov, MatrixXf proc_noise, MatrixXf meas_noise);

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
        void update(MatrixXf z);

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
         *
         * Parameters:
         * sigmas (Matrix): either the process or measurement sigma Matrix depending on whether we're in the predict or update fxn
         * wm, wc (Row Vectors): weights for the covariance and mean sigma points
         * r_q (Matrix): R (measurement noise) or Q (process noise) matrix depending on if we're updating or predicting
         * xp_zp (vector): Prior mean or measurement mean vectors (passed in by pointer to update)
         * Pp_Pz (Matrix): Prior covariance (Pp) or measurement covariance matrix (passed in by pointer to update)
         */
        void unscented_transform(MatrixXf sigmas, RowVectorXf wm, RowVectorXf wc, MatrixXf r_q, VectorXf* xp_zp, MatrixXf* Pp_Pz);
    
    private:
        int s_dim, m_dim, c_dim; //dimensions that get updated with parameter values
        int num_sigmas; //number of sigma points to generate
        float dt; //time step

        /*sigma parameters*/
        float beta, kappa, alpha, lambda;

        /*Vectors and Matricies*/
        
        //Vectors
        //state_est: vector of dimension s_dim. Stores the estimate of the state (mean). Updated whenever the predict-update sequence is called. Denoted as "x" in literature
        //control: vector of dimension c_dim. Not always used, but stores whatever control signal is passed to the system. Denoted as "u" in literature
        //data: vector of dimension m_dim. Stores data from sensors/signals. Denoted as "z" in literature
        //residual: vector that's updated whenever update is called. The error between the data points and their calculated mean
        VectorXf state_est, control, data, residual;
        RowVectorXf W_mean, W_cov; //Row vectors that hold the weights for the mean and covariance as calculated in Van Der Merwe's paper

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
        //control_mat: Optional matrix used for storing the transition function for systems with control inputs. <dim_s rows, dim_c cols>
        MatrixXf state_cov, meas_cov, meas_covI, meas_err, proc_err, cross_cov, gain, sigmas_s, sigmas_m, control_mat;


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