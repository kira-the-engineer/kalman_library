#include "kalman.hpp"
#include <Arduino.h>

using namespace Eigen;

KF::KF(int x, int z, int u){
    //set dimensions to parameters
    this->dim_x = x;
    this->dim_z = z;
    this->dim_u = u;

    //initialize identity matrix
    this->I.setIdentity(this->dim_x, this->dim_x);

    //Initialize system matricies
    this->x.Zero(this->dim_x, 1);
    this->P.setIdentity(this->dim_x, this->dim_x);
    this->Q.setIdentity(this->dim_x, this->dim_x);
    this->F.setIdentity(this->dim_x, this->dim_x);
    this->R.setIdentity(this->dim_z, this->dim_z);
    
    this->B.Zero(this->dim_x, this->dim_u);
    this->z.Zero(this->dim_z, 1);
    this->y.Zero(this->dim_z, 1);
    this->K.Zero(this->dim_x, this->dim_z);
    this->S.Zero(this->dim_z, this->dim_z);
    this->SI.Zero(this->dim_z, this->dim_z);
    this->u.Zero(this->dim_u, 1);
}

KF::KF(){}

KF::~KF(){}

//setters
void KF::set_K(MatrixXf c)
{
    this->K = c;
}

void KF::init(MatrixXf x, MatrixXf R, MatrixXf P, MatrixXf H, MatrixXf Q, MatrixXf F)
{
    //set an initial value for the state vector
    this->x = x;

    //initialize measurement noise
    this->R = R;

    //initialize process covariance
    this->P = P;

    //initialize measurement function
    this->H = H;

    //initialize processes noise
    this->Q = Q;

    //initialize process matrix/system matrix
    this->F = F;
}

void KF::update(MatrixXf z){
    //calculate the residual
    this->y = z - H * x;

    //S = HPH' + R -> Update the system uncertainty
    this->S = H*P*H.transpose() + R;

    //get inverse of S for kalman gain
    this->SI = S.inverse();

    //calculate the kalman gain
    this->K = P*H.transpose()*SI;

    //calculate new x
    this->x = x + K*y;

    //P = (I-KH)P(I-KH)' + KRK' <- this is more numerically stable than just I - (KH)P
    this->P = (I-K*H)*P*(I-K*H).transpose() + K*R*K.transpose();

    Serial.println("Updated State:");
    print_mtxf_arduino(this->x);
    Serial.println("Updated Covariance:");
    print_mtxf_arduino(this->P); 

}

void KF::predict(){
    Serial.println();
    Serial.println("Predict");
    //calculate the prior x
    this->x = this->F*this->x + this->B*this->u;
    //print_mtxf_arduino(this->x);

    //calculate the prior P
    this->P = this->F*this->P*this->F.transpose() + this->Q;
    //print_mtxf_arduino(this->P);
}


//The below function comes from the bolder flight eigen example by RandomVibe
// PRINT MATRIX (float type)
// By: randomvibe
//-----------------------------
void KF::print_mtxf_arduino(Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   nrow = X.rows();
   ncol = X.cols();
   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

void KF::print_matrices()
{
    Serial.println("Current state vector (x):");
    print_mtxf_arduino(this->x);
    Serial.println("Last measurement used (z):");
    print_mtxf_arduino(this->z);
    Serial.println("Residual (y):");
    print_mtxf_arduino(this->y);
    Serial.println("State Covariance (P):");
    print_mtxf_arduino(this->P);
    Serial.println("Process Noise (Q):");
    print_mtxf_arduino(this->Q);
    Serial.println("State Transition Matrix (F):");
    print_mtxf_arduino(this->F);
    Serial.println("Measurement Noise (R):");
    print_mtxf_arduino(this->R);
    Serial.println("Measurement Matrix (H):");
    print_mtxf_arduino(this->H);
    Serial.println("Kalman Gain (K):");
    print_mtxf_arduino(this->K);
    Serial.println("System uncertainty and inverse (S):");
    print_mtxf_arduino(this->S);
    print_mtxf_arduino(this->SI);
    Serial.println("Control input vector (u):");
    print_mtxf_arduino(this->u);
    Serial.println("Control transition matrix (B):");
    print_mtxf_arduino(this->B);
}
