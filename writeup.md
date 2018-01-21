**Extended Kalman Filter Project**


Self-Driving Car Engineer Nanodegree Program

In this project it is utilized a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

[//]: # (Image References)
[image1]: ./Docs/run_dataset1.png
[image2]: ./Docs/run_dataset2.png

### Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./ExtendedKF
```

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x ["estimate_y"] <= kalman filter estimated position y ["rmse_x"] ["rmse_y"] ["rmse_vx"] ["rmse_vy"]


## [Rubric](https://review.udacity.com/#!/rubrics/748/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Compiling

#### 1. Code must compile without errors with cmake and make.

As the file ```CMakeLists.txt``` was made as general as possible (including necessary steps for compiling in Mac, which is the platform I used to work in this project) I left if unchanged. Thus, after setting up [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), the compilation and building can be done using the standard procedure (from the project's top directory):

```
mkdir build
cd build
cmake ..
make
./ExtendedKF
```


### Accuracy

#### 1. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt which is the same data file the simulator uses for Dataset 1".

Running the algorithm agains Dataset 1 (see following screenshot), in the simulator produced the RMSE [0.0974, 0.0855, 0.4517, 0.4404], which is less than the required pass mark.
![image1]

When running agains Dataset 2 (see following screenshot) the RMSE was [0.0726, 0.0965, 0.4219, 0.4937], which is also less than the required pass mark.
![image2]


### Follows the Correct Algorithm

#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The implementation follows the steps learned during the lessons, following the formulas to first predict and then update.

To do the prediction the formula used is the following (where ```Ft``` is the transpose of ```F```):
```
x' = F * x
P' = F * P * Ft + Q
```

To do the update for laser measurements the base formula for KF is used, which is the following (where ```Ht``` is the transpose of ```H``` and ```Si``` is the inverse of ```S```):
```
y = z - H * x'
S = H * P * Ht + R
K = P * Ht * Si

x = x' + (K * y)
P = (I - K * H) * P
```

To do the upgrade for radar measurements, the adapted formula for EKF is used, which is the following (where ```h()``` is the function to transform from polar to cartesian, ```Hj``` is calculated Jacobian respect to ```x'```,  ```Hjt``` is the transpose of ```Hj``` and ```Si``` is the inverse of ```S```):
```
y = z - h(x');
S = Hj * P * Hjt + R
K = P * Hjt * Si

x = x' + (K * y)
P = (I - K * Hj) * P
```

#### 2. Your Kalman Filter algorithm handles the first measurements appropriately.

The first measurement is used to initialize the state vector, in the case where the first measure is a polar coordinate it converted to cartesian with position and velocity components.

Covariance matrices can actually be initialized in the constructor, in the constructor the initial states for P, F and Q are defined, and fixed values for R (one for laser and one for radar) and H. It is worth to mention that the fixed value for H is only used for laser, in the case of radar, Hj (the Jacobian) is calculated during the update step.



#### 3. Your Kalman Filter algorithm first predicts then updates.

Upon receiving a measurement after the first, the algorithm applies the previously mentioned formulas to in each cycle first to predict the object's position to the current timestep (before applying the formula the matrices F and Q are updated according to the time difference), and then update the prediction using the new measurement.



#### 4. Your Kalman Filter can handle radar and lidar measurements.

The implemented algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type, basically two methods for updating are used, one for laser which used the standard KF algorithm, and for radar the adaptation of EKF is used which involves the conversion from cartesian to polar of the current state and calculating the Jacobian to be used instead of H in the process.


### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.

Modifications to the formulas were made in order to do some common calculations just once. Also the code was tried to be simplified to avoid unnecessary loops, data structures, complex control flow or code redundancy.

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.

One of the problems I faced was when doing the update for radar measurements, in the case where one measure was about to complete a lap, for example one measure gives an angle of 3.14 (near pi) and the next one an angle of -3.14 (near -pi) in radial terms they were very close, but numerically the difference was big producing bad results when updating the position. The solution I found was to do a normalization when converting the current state to polar coordinates based on the measure given by the radar. For example if the current state has an angle of 3.14 and the measure gets one of -3.14, instead of using 3.14 using -3.1431.., i.e. resting 2*pi. Basically the normalization consist in adding or resting 2*pi (which is a full lap) in order to express the angle numerically as closer as possible to the new measurement. The results obtained were very good and the algorithm was able to perform well in both datasets.

The other problem I found was to prevent the division by zero when doing conversion and calculating the Jacobian, in practical terms this only would occur when the measure gives the same coordinated of the origin object (which would be a very strange case), however is always good to be prepared for that case. The solution I found was to throw an exception which would be catch by the updating method, in which case would skip the procedure since in would be an inconsistent measurement and in the following cycle try to recover, it seems that this mechanism can introduce some fault tolerance without giving a random result which could lead to a greater loss of accuracy.
