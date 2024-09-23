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
    this->x.setZero(this->x_dim);
    this->z.setZero(this->z_dim);
    this->u.setZero(this->u_dim);
    this->y.setZero(this->z_dim);

    //row vector init
    this->Wc.setZero(1, this->num_sigmas);
    this->Wm.setZero(1, this->num_sigmas);

    //Weights and sigmas initialization
    this->set_weights(); //initializes the covariance and mean weight row vectors

    //initialize zero matricies for storing the transformed sigma points
    this->sigmas_f.setZero(this->num_sigmas, this->x_dim);
    this->sigmas_h.setZero(this->num_sigmas, this->z_dim);

    //set function pointers 
    this->f_x = process;
    this->h_x = meas;
    this->nl_add = nl_add;
    this->nl_sub = nl_sub;

    //Set matricies
    this->P.setIdentity(this->x_dim, this->x_dim);
    this->Q.setIdentity(this->x_dim, this->x_dim);
    this->R.setIdentity(this->z_dim, this->z_dim);
    this->K.setZero(this->x_dim, this->z_dim);
    this->B.setZero(this->x_dim, this->u_dim);
    this->S.setZero(this->z_dim, this->z_dim);
    this->SI.setZero(this->z_dim, this->z_dim);   

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

void UKF::set_weights(){
    this->lambda = powf(this->alpha, 2) * (this->x_dim + this->kappa) - this->x_dim;
    this->Wc.fill(0.5 / (this->x_dim + this->lambda)); //Wc[i] = 1/(2(n + lambda)) where i = 1..2n
    this->Wm.fill(0.5 / (this->x_dim + this->lambda)); //Wm[i] = 1/(2(n + lambda)) where i = 1..2n
    this->Wc(0) = (this->lambda / (this->x_dim + this->lambda)) + (1.0 - powf(this->alpha, 2) + this->beta); //Wc[0] = lambda/(n + lambda) + 1 - alpha^2 + beta
    this->Wm(0) = this->lambda / (this->x_dim + this->lambda); //Wm[0] = lambda/(n + lambda)
}

MatrixXf UKF::generate_sigmas(VectorXf x, MatrixXf P){
    MatrixXf sigmas, chol;
    VectorXf row_i; //vectors to store chol(i)
    sigmas.setZero(this->num_sigmas, this->x_dim); //initialize empty matrix of (2n+1, n) where n is the dim of the state

    chol = P * (this->lambda + this->x_dim);
    chol = chol.llt().matrixU(); //take sqrt (cholesky decomp) of (n+lamba)P and return an upper triangular view

    //irst row of the sigma point matrix is the means
    sigmas.row(0) = x.col(0); //assigns row 0 to elements in the column vector x
    

    if(this->sub == true) { //if nonlinear values are in the matrices/vectors
        for(int i = 0; i < this->x_dim; i++){
            sigmas.row(i + 1) = this->nl_sub(-x, -row_i);
            sigmas.row(i + this->x_dim + 1) = this->nl_sub(x, row_i);
        }
    }
    else{
        for(int i = 0; i < this->x_dim; i++){
            row_i = chol.row(i);
            sigmas.row(i + 1) = x.col(0) + row_i;
            sigmas.row(i + this->x_dim + 1) = x.col(0) - row_i;
        }
    }

    return sigmas;
}