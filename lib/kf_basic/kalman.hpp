//A class for implementing a linear kalman filter on a microcontroller using Eigen
//This code is inspired by the FilterPy library and heavily references the concepts explained in "Kalman and Bayesian Filters in Python" by Roger Labbe
//https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

#include "eigen.h"
#include <math.h>
#include <Eigen/LU>

using namespace Eigen;

class KF{
    public:
        KF(int size_x, int size_z, int size_u); //constructor that takes in desired system dimensions as parameters
        KF(int size_x, int size_z, int size_u, MatrixXf B, MatrixXf u); //constructor that allows for intialization of control inputs
        KF(); //empty constructor
        ~KF();

        void update(MatrixXf z); //takes in new measurement and updates state 
        void predict();
        void print_mtxf_arduino(MatrixXf& K);
        void print_matrices(); // prints members of KF class
        void init(MatrixXf x, MatrixXf R, MatrixXf P, MatrixXf H, MatrixXf Q, MatrixXf F);

        //setters
        void set_K(MatrixXf copy); //for circumstances where you'd want to hardcode the kalman gain to reduce computational complexity


    protected:
        //system dimensions
        // x (int): number of state variables for the KF.
        // z (int): number of measurement variables
        // u (int): size of control vector- if used
        int dim_x, dim_z, dim_u;

        //n-size identity matrix
        MatrixXf I;

        //System Matricies
        //x: current state estimate vector <x rows, 1 col>. Any call to the update or predict updates this variable
        //P: current state covariance matrix <x rows, x cols>. Any call to the update or predict updates this variable
        //z: last measurement used in update
        //R: measurement noise matrix <z rows, z cols>
        //F: state transition matrix (Process Model)
        //Q: process noise matrix
        //H: Measurement function <z rows, x cols>
        //y: Residual of update
        //K: Kalman gain <x rows, z cols>
        //S: System uncertainty (P projected to measurement space)
        //SI: System uncertainty Inverse
        //B: control transition matrix <x rows, u cols>, default is a zero matrix of size dim_x, dim_u
        //u: control vector, default is a zero vector of size dim_u, 1
        MatrixXf x, z, y, P, R, Q, F, H, K, S, SI, B, u;
};