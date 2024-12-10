#include "ukf.hpp"

UKF::UKF(int x, int z, float time, float a, float b, MatrixXf (*process)(MatrixXf, float), MatrixXf (*meas)(MatrixXf), float k = 0.0, int u = 1){
    //set dims first
    this->s_dim = x;
    this->m_dim = z;
    this->c_dim = u;
    this->num_sigmas = 2*this->s_dim + 1;

    /*set floats*/
    this->dt = time;
    this->alpha = a;
    this->beta = b;
    this->kappa = k;

    //Vector initialization
    this->state_est.setZero(this->s_dim); //state est vector
    this->data.setZero(this->m_dim); //measurements vector
    this->control.setZero(this->c_dim); //control vector
    this->residual.setZero(this->m_dim); //residual

    //row vector init
    this->W_mean.setZero(1, this->num_sigmas);
    this->W_cov.setZero(1, this->num_sigmas);


    //Weights and sigmas initialization
    this->set_weights(); //initializes the covariance and mean weight row vectors

    //initialize zero matricies for storing the transformed sigma points
    this->sigmas_s.setZero(this->num_sigmas, this->s_dim);
    this->sigmas_m.setZero(this->num_sigmas, this->m_dim);

    //set function pointers 
    this->f_x = process;
    this->h_x = meas;

    //Set matricies
    this->state_cov.setIdentity(this->s_dim, this->s_dim);
    this->proc_err.setIdentity(this->s_dim, this->s_dim);
    this->meas_cov.setZero(this->m_dim, this->m_dim);
    this->meas_covI.setZero(this->m_dim, this->m_dim);
    this->meas_err.setIdentity(this->m_dim, this->m_dim);
    this->gain.setZero(this->s_dim, this->m_dim);
    this->control_mat.setZero(this->s_dim, this->c_dim);
    this->cross_cov.setZero(this->s_dim, this->m_dim);
}

UKF::UKF(){
    //default constructor
}

UKF::~UKF(){
    //deconstructor
}

void UKF::init(VectorXf x, MatrixXf P, MatrixXf Q, MatrixXf R){
    this->state_est = x;
    this->state_cov = P;
    this->proc_err = Q;
    this->meas_err = R;
}

void UKF::init_nonlinear(VectorXf (*add)(VectorXf, VectorXf) = NULL, VectorXf (*sub)(VectorXf, VectorXf) = NULL, VectorXf (*ux)(MatrixXf, RowVectorXf) = NULL, VectorXf (*uz)(MatrixXf, RowVectorXf) = NULL) {
    this->nl_add = add;
    this->nl_sub = sub;
    this->state_mean = ux;
    this->meas_mean = uz;
}

void UKF::set_weights(){
    this->lambda = powf(this->alpha, 2) * (this->s_dim + this->kappa) - this->s_dim;
    this->W_cov.fill(0.5 / (this->s_dim + this->lambda)); //Wc[i] = 1/(2(n + lambda)) where i = 1..2n
    this->W_mean.fill(0.5 / (this->s_dim + this->lambda)); //Wm[i] = 1/(2(n + lambda)) where i = 1..2n
    this->W_cov(0) = (this->lambda / (this->s_dim + this->lambda)) + (1.0 - powf(this->alpha, 2) + this->beta); //Wc[0] = lambda/(n + lambda) + 1 - alpha^2 + beta
    this->W_mean(0) = this->lambda / (this->s_dim + this->lambda); //Wm[0] = lambda/(n + lambda)
}

MatrixXf UKF::generate_sigmas(VectorXf x, MatrixXf P){
    MatrixXf sigmas, chol;
    VectorXf row_i; //vectors to store chol(i)
    sigmas.setZero(this->num_sigmas, this->s_dim); //initialize empty matrix of (2n+1, n) where n is the dim of the state

    chol = P * (this->lambda + this->s_dim);
    chol = chol.llt().matrixU(); //take sqrt (cholesky decomp) of (n+lamba)P and return an upper triangular view

    //first row of the sigma point matrix is the means
    sigmas.row(0) = x.col(0); //assigns row 0 to elements in the mean vector


    if(this->nl_sub != NULL) { //if nonlinear values are in the matrices/vectors
        for(int i = 0; i < this->s_dim; i++){
            row_i = chol.row(i);
            sigmas.row(i + 1) = this->nl_sub(-x, -row_i);
            sigmas.row(i + this->s_dim + 1) = this->nl_sub(x, row_i);
        }
    }
    else{
        for(int i = 0; i < this->s_dim; i++){
            row_i = chol.row(i);
            sigmas.row(i + 1) = x.col(0) + row_i;
            sigmas.row(i + this->s_dim + 1) = x.col(0) - row_i;
        }
    }

    return sigmas;
}