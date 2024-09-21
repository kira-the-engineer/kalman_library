#include "ukf.hpp"

UKF::UKF(int x, int z, float time, float a, float b, MatrixXf (*process)(MatrixXf, float), MatrixXf (*meas)(MatrixXf), 
    VectorXf (*nl_add)(VectorXf, VectorXf) = NULL, VectorXf (*nl_sub)(VectorXf, VectorXf) = NULL, float k = 0.0, int u = 1){
    //set dims first
    this->x_dim = x;
    this->z_dim = z;
    this->u_dim = u;
    this->num_sigmas = 2*this->x_dim + 1;

    /*set floats*/
    this->dt = time;
    this->alpha = a;
    this->beta = b;
    this->kappa = k;

    //Vector initialization
    this->x.Zero(this->x_dim);
    this->z.Zero(this->z_dim);
    this->u.Zero(this->u_dim);
    this->y.Zero(this->z_dim);

    //Weights and sigmas initialization
    this->set_weights(this->alpha, this->beta, this->kappa, this->x_dim); //initializes the covariance and mean weight row vectors

    //initialize zero matricies for storing the transformed sigma points
    this->sigmas_f.Zero(this->num_sigmas, this->x_dim);
    this->sigmas_h.Zero(this->num_sigmas, this->z_dim);

    //set function pointers 
    this->f_x = process;
    this->h_x = meas;
    this->nl_add = nl_add;
    this->nl_sub = nl_sub;

    //Set matricies
    this->P.setIdentity(this->x_dim, this->x_dim);
    this->Q.setIdentity(this->x_dim, this->x_dim);
    this->R.setIdentity(this->z_dim, this->z_dim);
    this->K.Zero(this->x_dim, this->z_dim);
    this->B.Zero(this->x_dim, this->u_dim);
    this->S.Zero(this->z_dim, this->z_dim);
    this->SI.Zero(this->z_dim, this->z_dim);   

    //set flags
    if(this->nl_add != NULL){
        this->add = true;
    }
    else{
        this->add = false;
    }
    if(this->nl_sub != NULL){
        this->sub = true;
    }
    else{
        this->sub = false;
    }
}

UKF::UKF(){
    //default constructor
}

UKF::~UKF(){
    //deconstructor
}

void UKF::set_weights(float a, float b, float k, int n){
    //note n is the dimensionality of the state vector!
    this->lambda = pow(a, 2) * (n + k) - n;
    this->Wc.fill(1. / (2*(n + this->lambda))); //Wc[i] = 1/(2(n + lambda)) where i = 1..2n
    this->Wm.fill(1. / (2*(n + this->lambda))); //Wm[i] = 1/(2(n + lambda)) where i = 1..2n
    this->Wc(0) = this->lambda / (n + this->lambda) + (1. - pow(a, 2) + b); //Wc[0] = lambda/(n + lambda) + 1 - alpha^2 + beta
    this->Wm(0) = this->lambda / (n + this->lambda); //Wm[0] = lambda/(n + lambda)
}

MatrixXf UKF::generate_sigmas(VectorXf x, MatrixXf P){
    MatrixXf sigmas, chol;
    VectorXf row_i; //vectors to store chol(i)
    sigmas.Zero(this->num_sigmas, this->x_dim); //initialize empty matrix of (2n+1, n) where n is the dim of the state

    chol = P * (this->lambda * this->x_dim);
    chol = chol.llt().matrixU(); //take sqrt (cholesky decomp) of (n+lamba)P and return an upper triangular view

    //first row of the sigma point matrix is the means
    sigmas.row(0) = x.col(0); //assigns row 0 to elements in the column vector x
    

    for(int i = 0; i < this->x_dim; i++){
        row_i = chol.row(i);
        sigmas.row(i + 1) = x.col(0) + row_i;
        sigmas.row(i + this->x_dim + 1) = x.col(0) - row_i;
    }

    return sigmas;
}