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

int main(){
   UKF ukf = UKF(2, 1, 1, .3, 2., f_x, h_x, NULL, NULL, 0.1, 1);

   MatrixXf sigma;
   Vector2f X(0., 0.);
   Matrix2f P {{32., 15.}, {15., 40.}};



   sigma = ukf.generate_sigmas(X, P);
   cout << "sigma" << endl << sigma << endl;
}