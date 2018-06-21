# 08. Project: Unscented Kalman Filters

## 1. List of files
* src/\*.cpp, src/\*.h : source codes of this project.
* output_dataset1.png: Screen shot of simulator result using dataset1.
* RMSE_pos_LidarRadar.png: RMSE convergences of positions of px and py.
* RMSE_v_LidarRadar.png: RMSE convergences of velocities of vx and vy.
* RMSE_transition_comparison.png: Comparison of RMSE convergences between UKF(Lidar+Radar), UKF(Lidar-only), UKF(Radar-only) and EKF(Lidar+Radar).

[//]: # (Image References)
[dataset1]: ./output_dataset1.png
[RMSEcomparison]: ./RMSE_comparison.png
[RMSEposLidarRadar]: ./RMSE_pos_LidarRadar.png
[RMSEvLidarRadar]: ./RMSE_v_LidarRadar.png

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## 2. Writeup / README
### 2-1. Compiling

#### 2-1-1. CRITERIA: Your code should compile.

My code successfully compiled with cmake and make using default CMakeLists.txt in the following environments:
* OS: Windows10 and Ubuntu 16.4 bash on WSL of Windows 10 Fall Creators Update.
* cmake v3.5.1
* GNU Make v4.1
* gcc/g++ v5.4.0

### 2-2. Accuracy

#### CRITERIA: px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt", which is the same data file the simulator uses for Dataset 1.

RMSE converged into values under the given criteria [.09, .10, .40, .30] with dataset1.

![dataset1][dataset1]
![RMSEposLidarRadar][RMSEposLidarRadar]
![RMSEvLidarRadar][RMSEvLidarRadar]
![RMSEcomparison][RMSEcomparison]

### 2-3. Follows the Correct Algorithm

#### 2-3-1. CRITERIA: Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

My Sensor Fusion algorithm is following the general processing flow of Unscented Kalman Filter algorithm which was taught in the lessons.

#### 2-3-2. CRITERIA: Your Kalman Filter algorithm handles the first measurements appropriately.

In "ukf.cpp", the state vector was initialized by using the first measurements. The initializing process was branched into different ones depending on a sensor types (Lidar or Radar) like this:

```cpp
  if (!is_initialized_)
  {
    // first measurement
    cout << "EKF: " << endl;
    x_ = VectorXd(5);
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double _ro = meas_package.raw_measurements_[0];
      double _theta = meas_package.raw_measurements_[1];
      double _ro_dot = meas_package.raw_measurements_[1];
      x_ << _ro*cos(_theta), _ro*sin(_theta), _ro_dot, 0.0, 0.0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
```

#### 2-3-3. CRITERIA: Your Kalman Filter algorithm first predicts then updates.

In "ukf.cpp", my UKF algorithm predicts the state vector at the time when the next measurement was processed, then updates the state vector by using the measurement result.

#### 2-3-4. CRITERIA: Your Kalman Filter can handle radar and lidar measurements.

In "ukf.cpp", my Kalman Filter algorithm handles both of radar and lidar measurements like this:

```cpp
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else
  {
    UpdateLidar(meas_package);
  }
```

### 2-4. Code Efficiency

#### 2-4-1. CRITERIA: Your algorithm should avoid unnecessary calculations..
My algorithm is avoiding unnecessary calculation as much as I can.

