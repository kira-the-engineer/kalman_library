#include "eigen.h"
#include "math.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <functional>

using namespace Eigen;
using namespace std;

class UKF {
    public:
        UKF(); //empty constructor
        /*
         * Constructor for a basic unscented kalman filter object.
         * Parameters:
         * dim_x (int): dimension of the state vector- pertains to the number of state variables used in the filter (i.e how many rows in the vector)
         * dim_z (int): dimension of the measurement vector- changes based on if you have multiple sensors and what each sensor outputs. For instance
         * if you have a single position sensor with outputs (x,y) z would be a column vector with 2 rows. 
         * dim_u (int): dimension of the control vector, where number of rows is the number of control inputs. defaults to 1 by 1 vector
         * dt (float): time step used in the state transition process- in seconds
         * f_x (function): Function that moves the sigma points forward in time according to the process model. Should return a matrix of transformed 
         * sigma points. Takes in the sigma points matrix and the time step used to forward each point
         * h_x (function): Function responsible for converting the prior sigma points returned by f_x into measurements. Should return a matrix
         */ 
        UKF(int dim_x, int dim_z, int dim_u = 1, float dt, function<MatrixXf(MatrixXf, float)>f_x, function<MatrixXf(MatrixXf)>h_x);
        /*
         * Predict
         */
        void predict();

        /*
         * Update
         */
        void update(MatrixXf z);
    
    private:
        int x_dim, z_dim, u_dim; //dimensions that get updated with parameter values
        int num_sigmas = 2*x_dim + 1; //from Van der Merwe's paper on the UKF
        /*Vectors and Matricies*/
        VectorXf x, z, u; //state, measurement, and control vectors
        //P: current state covariance matrix <x rows, x cols>. Any call to the update or predict updates this variable
        //R: measurement noise matrix <z rows, z cols>
        //Q: process noise matrix
        //K: Kalman gain <x rows, z cols>
        //S: System uncertainty (P projected to measurement space)
        //SI: System uncertainty Inverse
        //B: control transition matrix <x rows, u cols>, default is a zero matrix of size dim_x, dim_u
        //u: control vector, default is a zero vector of length dim_u
        //Wm: Matrix of mean weights for sigma points as calculated using Van der Merwe's method for generating sigmas
        //Wc: Matrix of covariance weights for sigma points as calculated using Van der Merwe's method for generating sigmas
        //sigmas_f: Matrix that stores sigma points forwarded in time by f_x, of dimensions (2*dim_x + 1), dim_x
        //sigmas_h: Matrix that stores sigma points projected to the measurement domain by h_x, of dimensions (2*dim_x + 1), dim_z
        //y: residual
        MatrixXf P, R, Q, K, S, SI, B, u, Wm, Wc, sigmas_f, sigmas_h, y;

        
};