
This project implements kalman filters to predict position and velocity for constant velocity model. 
Data input comes from both radar and lidar sensors, for radar data linear approximation using partial 
derivatives(jacobian) is used to apply kalman filters. 

RMSE values:

Dataset1:
X: 0.0973
Y: 0.0855
VX: 0.4513
VY: 0.4399

Dataset2:
X: 0.0726
Y: 0.0967
VX: 0.4579
VY: 0.4966

Visualization for dataset1 using utilities mentioned in project:
https://plot.ly/create/?fid=hkg:34
Visulaization for dataset2:
https://plot.ly/create/?fid=hkg:32

#build instructions 

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

To run have simulator up and listening on default port, connect using the following:
./ExtendedKF ../data/<input data>  ><output capture>
---

## Other Important Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

=======
# KalmanFilters_CVModel
constant velocity model

