/*
 * File strictly for debugging/testing the UKF lib functions for development purposes. This allows for work to be done without the microcontroller
 * being plugged in. This should not be used when wanting to benchmark the actual hardware performance of the filters.
 */

#include "ukf.hpp"
#include <iostream>

MatrixXf f_x(MatrixXf x, float dt) {
   MatrixXf xout;

   return xout;
}

MatrixXf h_x(MatrixXf x) {
   MatrixXf hout;

   return hout;
}

VectorXf sub(VectorXf a, VectorXf b) {
   return a - b;
}

RowVectorXf nl_xy(float x, float y)   {
   RowVector2f out;
   out << x + y, .1 * pow(x, 2) + y * y;
   return out;
}

int main(){
   Matrix<float, 2, 1> X{{0.}, {0.}};
   Matrix2f P {{32., 15.}, {15., 40.}};
   Matrix2f Q {{0., 0.}, {0., 0.}};
   Matrix2f R {{0, 0}, {0, 0}};

   UKF ukf = UKF(X, P, Q, R, f_x, h_x);
}