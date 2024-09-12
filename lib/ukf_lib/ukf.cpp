#include "ukf.hpp"

UKF::UKF(int x, int z, int u = 1, float time, MatrixXf (*process)(MatrixXf, float), MatrixXf (*meas)(MatrixXf)){
    //set dims first
    this->x_dim = x;
    this->z_dim = z;
    this->u_dim = u;
    this->num_sigmas = 2*this->x_dim + 1;
    this->dt = time;

    //Vector initialization
    this->x.Zero(this->x_dim);
    this->z.Zero(this->z_dim);
    this->u.Zero(this->u_dim);
    this->y.Zero(this->z_dim);

    //Weights and sigmas initialization
    this->set_weights(); //initializes the covariance and mean weight row vectors

    //initialize zero matricies for storing the transformed sigma points
    this->sigmas_f.Zero(this->num_sigmas, this->x_dim);
    this->sigmas_h.Zero(this->num_sigmas, this->z_dim);

    //set function pointers 
    this->f_x = process;
    this->h_x = meas;

    //Set matricies
    this->P.setIdentity(this->x_dim, this->x_dim);
    this->Q.setIdentity(this->x_dim, this->x_dim);
    this->R.setIdentity(this->z_dim, this->z_dim);
    this->K.Zero(this->x_dim, this->z_dim);
    this->B.Zero(this->x_dim, this->u_dim);
    this->S.Zero(this->z_dim, this->z_dim);
    this->SI.Zero(this->z_dim, this->z_dim);   
}

UKF::UKF(){
    //default constructor
}

UKF::~UKF(){
    //deconstructor
}