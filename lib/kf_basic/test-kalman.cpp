#include <Arduino.h>
#include "kalman.hpp"

void setup() {
   Serial.begin(115200);
   while (!Serial) {}

   KF KF(2, 1, 0); //create KF

   Matrix2f P {{500., 0.}, {0., 49.}}; //initial covariance estimate
   Vector2f x {10., 4.5}; //intial state estimate
   Matrix2f F {{1., 1.}, {0., 1.}}; //Process model
   MatrixXf H {{1.0, 0.0}}; //Measurement model
   Matrix2f Q {{0.003, 0.005}, {0.005, 0.01}}; //Process noise
   MatrixXf R {{10.}}; //Measurement noise

   KF.init(x, R, P, H, Q, F);

   //simulated measurements for a "dog tracker"
   float zs [] = {5.149, -0.67, 1.682, 8.359, 6.92, 3.965, 7.562, 12.481, 4.108, 9.504, 10.908, 12.667, 15.752, 13.828, 
   18.739, 19.99, 18.531, 19.017, 23.278, 17.531, 19.646, 27.3, 25.764, 24.35, 25.279, 25.509, 30.114, 30.789, 29.292, 
   24.149, 29.882, 30.941, 41.846, 35.893, 30.521, 32.702, 41.294, 38.97, 35.436, 41.388, 38.02, 45.37, 44.643, 45.044,
   47.595, 45.869, 48.74, 46.58, 52.872, 52.102};

   for (float z : zs){ //for each measurement in the measurement array
      //predict
      KF.predict();

      //cast to matrix for scalar case (z can be multidimensional if you have multiple sensors!!)
      Matrix<float, 1, 1> temp_z {z};

      //update (pass 1x1 matrix in)
      KF.update(temp_z);
   }
}

void loop() {}