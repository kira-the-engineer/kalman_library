#include "ukf.hpp"
#include <Arduino.h>

MatrixXf f_x(MatrixXf x, float dt) {
   MatrixXf xout;

   return xout;
}

MatrixXf h_x(MatrixXf x) {
   MatrixXf hout;

   return hout;
}

RowVectorXf nl_xy(float x, float y)   {
   RowVector2f out;
   out << x + y, .1 * pow(x, 2) + y * y;
   return out;
}

void setup() {
   Serial.begin(9600);
   while (!Serial) {}
   //linear example for testing
   Vector<float, 2> X{{0.}, {0.}};
   Matrix2f P {{32., 15.}, {15., 40.}};
   Matrix2f Q {{0., 0.}, {0., 0.}};
   Matrix2f R {{0, 0}, {0, 0}};

   UKF ukf = UKF(X, P, Q, R, f_x, h_x);

   Matrix<float, 2 * STATE_DIM + 1, STATE_DIM> sigmas;
   sigmas = ukf.generate_sigmas(X, P);

   Serial.println("Sigma points");
   for (int i=0; i<sigmas.rows(); i++)
   {
       for (int j=0; j<sigmas.cols(); j++)
       {
           Serial.print(sigmas(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println(); 

   Matrix<float, 2 * STATE_DIM + 1, STATE_DIM>sigmas_f;
   sigmas_f.setZero();
   for(int i = 0; i < 2 * STATE_DIM + 1; i++){
      sigmas_f(i, 0) = sigmas(i, 0) + sigmas(i, 1);
      sigmas_f(i, 1) = .1 * powf(sigmas(i, 0), 2) + sigmas(i, 1) * sigmas(i, 1);
   }

   Serial.println("Transformed sigmas");
   for (int i=0; i< 2 * STATE_DIM + 1; i++)
   {
       for (int j=0; j < STATE_DIM; j++)
       {
           Serial.print(sigmas_f(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();


   ukf.unscented_transform(X, P, sigmas_f, Q, NULL, NULL);


   Serial.println("Transformed Mean");
   for (int i=0; i<X.rows(); i++)
   {
       for (int j=0; j<X.cols(); j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();



   Serial.println("Transformed covariance");
   for (int i=0; i<P.rows(); i++)
   {
       for (int j=0; j<P.cols(); j++)
       {
           Serial.print(P(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println(); 

}

void loop() {}
