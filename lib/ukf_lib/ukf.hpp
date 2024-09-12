#include "eigen.h"
#include "math.h"
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
         * dim_x (int): dimension of the state vector- pertains to the number of state variables used in the filter (i.e how many rows in the vector)
         * dim_z (int): dimension of the measurement vector- changes based on if you have multiple sensors and what each sensor outputs. For instance
         * if you have a single position sensor with outputs (x,y) z would be a column vector with 2 rows. 
         * dim_u (int): dimension of the control vector, where number of rows is the number of control inputs. defaults to 1 by 1 vector
         * dt (float): time step used in the state transition process- in seconds
         * process (function): Function that moves the sigma points forward in time according to the process model. Should return a matrix of transformed 
         * sigma points. Takes in the sigma points matrix and the time step used to forward each point
         * meas (function): Function responsible for converting the prior sigma points returned by f_x into measurements. Should return a matrix
         */ 
        UKF(int dim_x, int dim_z, int dim_u, float time, MatrixXf (*process)(MatrixXf, float), MatrixXf (*meas)(MatrixXf));

        ~UKF(); //deconstructor

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
         * x (vector): n-length vector that holds the mean/state of the filter
         * P (matrix): Covariance of filter
         * Returns:
         * sigmas- Matrix of size (dim_x, 2*dim_x+1). Each col contains sigma points for 1 dimension of the problem space
         */
        MatrixXf generate_sigmas(VectorXf x, MatrixXf P);
    
    private:
        int x_dim, z_dim, u_dim; //dimensions that get updated with parameter values
        int num_sigmas; //number of sigma points to generate
        float dt; //time step

        /*Vectors and Matricies*/
        VectorXf x, z, u, y; //state, measurement, control, and residual vectors
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
        MatrixXf P, R, Q, K, S, SI, B, sigmas_f, sigmas_h;
        RowVectorXf Wc, Wm;
    
        //function pointers
        MatrixXf (*f_x)(MatrixXf, float) = NULL;
        MatrixXf (*h_x)(MatrixXf) = NULL;

        void set_weights(); // Calculates weights according to Van Der Merwe's paper- should only be called in constructor

        
};