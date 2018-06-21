# 15. Project: Kidnapped Vehicle

## 1. Files and environments
### 1-1. List of files
* src/\*.cpp, src/\*.h : source codes of this project.
* output.png: Screen shot of simulator result.
* weight_history.jpg: History of weight.
* output.txt: Output of weight history.

### 1-2. Environment
My code successfully compiled with cmake and make using default CMakeLists.txt in the following environments:
* OS: Windows10 and Ubuntu 16.4 bash on WSL of Windows 10 Fall Creators Update.
* cmake v3.5.1
* GNU Make v4.1
* gcc/g++ v5.4.0

[//]: # (Image References)
[simulator_output]: ./output.png
[weight_history]: ./weight_history.jpg

---
## 2. Writeup / README

### 2-1. Accuracy
#### 2-1-1. CRITERIA: Does your particle filter localize the vehicle to within the desired accuracy?
My particle filter localize the vehicle to within the desired accuracy. The final output said "Success! Your particle filter passed!"
![simulator_output][simulator_output]

---
### 2-2. Performance
#### 2-2-1. CRITERIA: Does your particle run within the specified time of 100 seconds?
My particle filter run within the specified time of 100 seconds. The final output said "Success! Your particle filter passed!"
![weight_history][weight_history]

---
### 2-3. General
#### 2-3-1. CRITERIA: Does your code use a particle filter to localize the robot?
My code uses a particle filter to localize the robot in particle_filter.cpp.
