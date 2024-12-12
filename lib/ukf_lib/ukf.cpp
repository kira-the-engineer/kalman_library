#include "ukf.hpp"

UKF::UKF(MatrixXf state, MatrixXf cov, MatrixXf proc_noise, MatrixXf meas_noise, MatrixXf (*process)(MatrixXf, float), MatrixXf (*meas)(MatrixXf)){
    //initialize vectors first
    this->state = state; //set to intial value
    this->data.setZero(MEASUREMENT_DIM);
    this->residual.setZero(MEASUREMENT_DIM);

    //initialize weights
    this->set_weights();

    //initialize matrices for storing sigma points
    this->sigma_s.setZero(this->num_sigmas, STATE_DIM);
    this->sigmas_m.setZero(this->num_sigmas, MEASUREMENT_DIM);

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

// void UKF::init_nonlinear(VectorXf (*add)(VectorXf, VectorXf) = NULL, VectorXf (*sub)(VectorXf, VectorXf) = NULL, VectorXf (*ux)(MatrixXf, RowVectorXf) = NULL, VectorXf (*uz)(MatrixXf, RowVectorXf) = NULL) {
//     this->nl_add = add;
//     this->nl_sub = sub;
//     this->state_mean = ux;
//     this->meas_mean = uz;
// }

void UKF::set_weights(){
    this->lambda = powf(ALPHA, 2) * (STATE_DIM + KAPPA) - STATE_DIM;
    this->w_cov.fill(0.5 / (STATE_DIM + this->lambda)); //Wc[i] = 1/(2(n + lambda)) where i = 1..2n
    this->w_mean.fill(0.5 / (STATE_DIM + this->lambda)); //Wm[i] = 1/(2(n + lambda)) where i = 1..2n
    this->w_cov(0) = (this->lambda / (STATE_DIM + this->lambda)) + (1.0 - powf(ALPHA, 2) + BETA); //Wc[0] = lambda/(n + lambda) + 1 - alpha^2 + beta
    this->w_mean(0) = this->lambda / (STATE_DIM + this->lambda); //Wm[0] = lambda/(n + lambda)
}

// MatrixXf UKF::generate_sigmas(VectorXf x, MatrixXf P){
//     MatrixXf sigmas, chol;
//     VectorXf row_i; //vectors to store chol(i)
//     sigmas.setZero(this->num_sigmas, this->s_dim); //initialize empty matrix of (2n+1, n) where n is the dim of the state

//     chol = P * (this->lambda + this->s_dim);
//     chol = chol.llt().matrixU(); //take sqrt (cholesky decomp) of (n+lamba)P and return an upper triangular view

//     //first row of the sigma point matrix is the means
//     sigmas.row(0) = x.col(0); //assigns row 0 to elements in the mean vector


//     if(this->nl_sub != NULL) { //if nonlinear values are in the matrices/vectors
//         for(int i = 0; i < this->s_dim; i++){
//             row_i = chol.row(i);
//             sigmas.row(i + 1) = this->nl_sub(x, -row_i);
//             sigmas.row(i + this->s_dim + 1) = this->nl_sub(x, row_i);
//         }
//     }
//     else{
//         for(int i = 0; i < this->s_dim; i++){
//             row_i = chol.row(i);
//             sigmas.row(i + 1) = x + row_i;
//             sigmas.row(i + this->s_dim + 1) = x - row_i;
//         }
//     }

//     return sigmas;
// }

// void UKF::unscented_transform(VectorXf &m, MatrixXf &c, MatrixXf sigma, RowVectorXf wm, RowVectorXf wc, MatrixXf noise, VectorXf (*mean)(MatrixXf, RowVectorXf), VectorXf(*sub)(VectorXf, VectorXf)) {
//     //calculate the mean of the sigma points first
//     if(mean != NULL){
//         m = mean(sigma, this->W_mean);
//     }
//     else{
//         //find the inner product of the sigma points and the mean weights
//         m = wm * sigma; //this is really just equivalent to taking a dot product since we're multiplying a vector by a matrix. This is equivalent to the inner product
//     }

//     //now calculate the covariance
//     VectorXf y;
//     MatrixXf c_temp;
//     c_temp.setZero(this->num_sigmas, c.cols());
//     for(int i = 0; i < this->num_sigmas; i++){
//         if(sub != NULL){
//             y = nl_sub(sigma.row(i), m); //deal with nonlinear values in calculating the difference between the sigma points and the mean vector
//         }
//         else{
//             y = sigma.row(i) - m; //if values are linear, normal vector subtraction is just fine
//         }
//         c_temp = c_temp + (wc.col(i) * (y * y.transpose())); // Cov = Sum(cov_weight * ([sigmas[i] - mean][sigmas[i] - mean]^T)). Or the covariance is equal to the sum of the covariance weights multiplied by the outer product of
//                                               // the difference vector/residual between the sigma points and the mean
//     }
//     c_temp = c_temp + noise; //add in noise
//     c = c_temp;
// }

// void UKF::predict(){
// }

// void UKF::update(MatrixXf z){

// }