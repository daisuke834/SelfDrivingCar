# 17. Project: PID Control

## 1. Files and environments
### 1-1. List of files
* [src/\*.cpp, src/\*.h](./src/) : source codes of this project.
* [YouTube video](https://youtu.be/T8Dys44A53E): Screen shot video of simulator.
* [search.png](./search.png): History of hyper parameter search.
* [cte.png](./cte.png): History of Cross Track Error (CTE)
* [speed.png](./speed.png): History of vehicle speed.
* [steering.png](./steering.png): History of steering control.
* [throttle.png](./throttle.png): History of throttle control.

### 1-2. Environment
My code successfully compiled with cmake and make using default CMakeLists.txt in the following environments:
* OS: Windows10 and Ubuntu 16.4 bash on WSL of Windows 10 Fall Creators Update.
* cmake v3.5.1
* GNU Make v4.1
* gcc/g++ v5.4.0

[//]: # (Image References)
[search.png]: ./search.png
[cte.png]: ./cte.png
[speed.png]: ./speed.png
[steering.png]: ./steering.png
[throttle.png]: ./throttle.png

---
## 2. Writeup / README

### 2-1. Compilation
#### 2-1-1. CRITERIA: Your code should compile.
My code was successfully compiled without errors.

---
### 2-2. Implementation
#### 2-2-1. CRITERIA: The PID procedure follows what was taught in the lessons.
- I used PID control algorithm for steering control.
- I used twiddle and other hyperparameter seach to tune coefficients of the PID control.
![search.png][search.png]

---
### 2-3. Reflection
#### 2-3-1. CRITERIA: Describe the effect each of the P, I, D components had in your implementation.
- 'P' component is contributing to control steering to keep the vehicle at the center of the road.
- 'I' component is supposed to suppress the systematic bias of the steering control. However, I set the coefficient of I component to zero in this project because there was no significant performance improvement by this component in my hyperparameter searching.
- 'D' component is contributing to suppress oscilations of the vehicle position.

#### 2-3-2. CRITERIA: Describe how the final hyperparameters were chosen.
I used twiddle and its derivations for the fine tuning of hyperparameters to choose the final hyperparameter.

---
### 2-4. Simulation
#### 2-4-1. CRITERIA: The vehicle must successfully drive a lap around the track.
The vehicle successfully drove a lap around the track. Please refer to [YouTube video](https://youtu.be/T8Dys44A53E).

![cte.png][cte.png]
![speed.png][speed.png]
![steering.png][steering.png]
![throttle.png][throttle.png]

