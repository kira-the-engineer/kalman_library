#include "ukf.hpp"

UKF::UKF(VectorXf state, MatrixXf cov, MatrixXf proc_noise, MatrixXf meas_noise, MatrixXf (*process)(MatrixXf, float), MatrixXf (*meas)(MatrixXf)){
    //initialize vectors first
    this->state = state; //set to intial value
    this->data.setZero(MEASUREMENT_DIM);
    this->residual.setZero(MEASUREMENT_DIM);

    //initialize weights
    this->set_weights();

    //initialize matrices for storing sigma points
    this->sigma_s.setZero(2 * STATE_DIM + 1, STATE_DIM);
    this->sigmas_m.setZero(2 * STATE_DIM + 1, MEASUREMENT_DIM);

    //initialize other matrices
    this->state_cov = cov;
    this->proc_err = proc_noise;
    this->meas_cov.setZero(MEASUREMENT_DIM, MEASUREMENT_DIM);
    this->meas_covI.setZero(MEASUREMENT_DIM, MEASUREMENT_DIM);
    this->meas_err = meas_noise;
    this->gain.setZero(STATE_DIM, MEASUREMENT_DIM);
    this->cross_cov.setZero(STATE_DIM, MEASUREMENT_DIM);

    //set function pointers
    this->f_x = process;
    this->h_x = meas;
}

UKF::UKF(){
    //default constructor
}

UKF::~UKF(){
    //deconstructor
}

void UKF::init_nonlinear(VectorXf (*add)(VectorXf, VectorXf) = NULL, VectorXf (*sub)(VectorXf, VectorXf) = NULL, VectorXf (*ux)(MatrixXf, RowVectorXf) = NULL, VectorXf (*uz)(MatrixXf, RowVectorXf) = NULL) {
    this->nl_add = add;
    this->nl_sub = sub;
    this->state_mean = ux;
    this->meas_mean = uz;
}

void UKF::set_weights(){
    this->lambda = powf(ALPHA, 2) * (STATE_DIM + KAPPA) - STATE_DIM;
    this->w_cov.fill(0.5 / (STATE_DIM + this->lambda)); //Wc[i] = 1/(2(n + lambda)) where i = 1..2n
    this->w_mean.fill(0.5 / (STATE_DIM + this->lambda)); //Wm[i] = 1/(2(n + lambda)) where i = 1..2n
    this->w_cov(0) = (this->lambda / (STATE_DIM + this->lambda)) + (1.0 - powf(ALPHA, 2) + BETA); //Wc[0] = lambda/(n + lambda) + 1 - alpha^2 + beta
    this->w_mean(0) = this->lambda / (STATE_DIM + this->lambda); //Wm[0] = lambda/(n + lambda)
}

MatrixXf UKF::generate_sigmas(VectorXf x, MatrixXf P){
    Matrix<float,  2 * STATE_DIM + 1, STATE_DIM> sigmas; //create empty matrix to store generated sigma points in
    Matrix<float, STATE_DIM, STATE_DIM> cholesky; //empty matrix to store the cholesky decomp result in
    Matrix<float, STATE_DIM, 1> row_i; //vector that's used to store each row of the cholesky decom matrix for operations

    cholesky = P * (this->lambda + STATE_DIM);
    cholesky = cholesky.llt().matrixU(); //take square root of the covariance matrix scaled by (n + lambda) where n is the dimension of the state. Returns an upper triangular view of the matrix

    //The first row of the sigma point matrix is the means
    sigmas.row(0) = x;
    
    if(this->nl_sub != NULL) { //if nonlinear values are in the matrices/vectors
        for(int i = 0; i < STATE_DIM; i++){
            row_i = cholesky.row(i);
            sigmas.row(i + 1) = this->nl_sub(x, -row_i);
            sigmas.row(i + STATE_DIM + 1) = this->nl_sub(x, row_i);
        }
    }
    else{
        for(int i = 0; i < STATE_DIM; i++){
            row_i = cholesky.row(i);
            sigmas.row(i + 1) = x + row_i;
            sigmas.row(i + STATE_DIM + 1) = x - row_i;
        }
    }

    return sigmas;
}

void UKF::unscented_transform(Ref<VectorXf> m, Ref<MatrixXf> c, MatrixXf sigma, MatrixXf noise, VectorXf (*mean)(MatrixXf, RowVectorXf), VectorXf(*sub)(VectorXf, VectorXf)) {
    //calculate the mean of the sigma points first
    if(mean != NULL){
        m = mean(sigma, this->w_mean);
    }
    else{
        //find the inner product of the sigma points and the mean weights
        m = this->w_mean * sigma;
    }
    
    MatrixXf cov_new(c.rows(), c.rows());
    cov_new.setZero();
    for(int i = 0; i < 2 * STATE_DIM + 1; i++){
        if(sub != NULL){
            cov_new += this->w_cov(i) * sub(sigma.row(i), m) * sub(sigma.row(i), m).transpose();
        }
        else{
            cov_new += this->w_cov(i) * (sigma.row(i) - m) * (sigma.row(i) - m).transpose();
        }
    }
    c = cov_new + noise;
}

// void UKF::predict(){
// }

// void UKF::update(MatrixXf z){

// }