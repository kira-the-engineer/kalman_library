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
   float zs [] = { 3.798,  5.752, -0.175, -1.439, -0.669, 10.708, 13.082,  8.879,  6.053,
                  16.593, 14.203, 17.94,  15.531, 15.244, 19.718, 16.934, 14.592, 21.949,
                  16.443, 16.889, 19.27,  23. ,22.403, 21.112, 31.612, 21.784, 28.652,
                  28.461, 26.627, 30.991, 30.352, 34.2, 34.377, 35.462, 39.094, 37.122,
                  31.182, 38.628, 39.586, 37.316, 46.669, 40.068, 41.256, 46.44, 39.371,
                  45.859, 50.519, 49.834, 48.092, 51.203};

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
