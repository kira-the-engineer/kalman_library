#include <Arduino.h>
#include "ukf.hpp"

MatrixXf f_x(MatrixXf x, float dt) {
   MatrixXf xout;

   return xout;
}

MatrixXf h_x(MatrixXf x) {
   MatrixXf hout;

   return hout;
}

void setup() {
   Serial.begin(9600);
   while (!Serial) {}
   //linear example for testing
   Matrix<float, 2, 1> X{{0.}, {0.}};
   Matrix2f P {{32., 15.}, {15., 40.}};
   Matrix2f Q {{0., 0.}, {0., 0.}};
   Matrix2f R {{0, 0}, {0, 0}};

   UKF ukf = UKF(X, P, Q, R, f_x, h_x);
}

void loop() {}
