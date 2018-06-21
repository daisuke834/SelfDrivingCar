# 06. Project: Extended Kalman Filters

## 1. List of files
* src/\*.cpp, src/\*.h : source codes of this project.
* output_dataset1.png: Screen shot of simulator result using dataset1.
* output_dataset2.png: Screen shot of simulator result using dataset2.
* RMSE_pos_LidarRadar.png: RMSE convergences of positions of px and py.
* RMSE_v_LidarRadar.png: RMSE convergences of velocities of vx and vy.
* RMSE_comparison.png: Comparison of final RMSE values at time step 499 between Lidar+Radar, Lidar-only and Radar-only.
* RMSE_transition_comparison.png: Comparison of RMSE convergences between Lidar+Radar, Lidar-only and Radar-only.

[//]: # (Image References)
[dataset1]: ./output_dataset1.png
[dataset2]: ./output_dataset2.png
[RMSEcomparison]: ./RMSE_comparison.png
[RMSEposLidarRadar]: ./RMSE_pos_LidarRadar.png
[RMSEvLidarRadar]: ./RMSE_v_LidarRadar.png
[RMSE_transition_comparison]: ./RMSE_transition_comparison.png

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

#### 2-2-1. CRITERIA: px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt which is the same data file the simulator uses for Dataset 1"

RMSE converged into values under the given criteria [.11, .11, 0.52, 0.52] with each of dataset1 and dataset2.

![dataset1][dataset1]
![dataset2][dataset2]
![RMSEposLidarRadar][RMSEposLidarRadar]
![RMSEvLidarRadar][RMSEvLidarRadar]

### 2-3. Follows the Correct Algorithm

#### 2-3-1. CRITERIA: Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

My Sensor Fusion algorithm is following the general processing flow of Extended Kalman Filter algorithm which was taught in the lessons.

#### 2-3-2. CRITERIA: Your Kalman Filter algorithm handles the first measurements appropriately.

In "FusionEKF.cpp", the state vector was initialized by using the first measurements. The initializing process was branched into different ones depending on a sensor types (Lidar or Radar) like this:

```cpp
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float _ro = measurement_pack.raw_measurements_[0];
      float _theta = measurement_pack.raw_measurements_[1];
      float _ro_dot = measurement_pack.raw_measurements_[1];
      ekf_.x_ << _ro*cos(_theta), _ro*sin(_theta), _ro_dot*cos(_theta), _ro_dot*sin(_theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;
```

#### 2-3-3. CRITERIA: Your Kalman Filter algorithm first predicts then updates.

In "FusionEKF.cpp", my Kalman Filter algorithm predicts the state vector at the time when the next measurement was processed, then updates the state vector by using the measurement result.

#### 2-3-4. CRITERIA: Your Kalman Filter can handle radar and lidar measurements.

In "FusionEKF.cpp", my Kalman Filter algorithm handles both of radar and lidar measurements like this:

```cpp
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
```

### 2-4. Code Efficiency

#### 2-4-1. CRITERIA: Your algorithm should avoid unnecessary calculations.

My algorithm is avoiding unnecessary calculation as much as I can.
I with I could read through "Google C++ Style Guide" (https://google.github.io/styleguide/cppguide.html) and could stick to it, but I didn't read it yet. This would be my next challenge during term2.

### 2-5. Other Efforts
I compared RMSE values between different sensor combinations (Lidar+Radar, Lidar only, and Radar only).
* Lidar and Radar: Best performance was gained by taking advantage of both of the sensors.
* Lidar only: px, py, vx and vy converge slower than the sensor fusion of Lidar and Radar. The convergence ov px is not stable.
* Radar only: px and py don't converge well due to its lower spacial resolution. Velocities of vx and vy converges well because Radar can measure velocity directly, but the RMSE of vx and vy is still higher than Lidar+Radar because Radar cannot measure the other component of the velocity which is orthogonal to the rho direction.

![RMSEcomparison][RMSEcomparison]
![RMSE_transition_comparison][RMSE_transition_comparison]


