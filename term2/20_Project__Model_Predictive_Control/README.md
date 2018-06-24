# 20. Project: Model Predictive Control

## 1. Files and environments
### 1-1. List of files
* [src/\*.cpp, src/\*.h](./src/) : source codes of this project.
* [YouTube video](https://youtu.be/Sg6Ga40LST8): Screen shot video of simulator.
* [Model](./mymodel.png) : My model for this project.

### 1-2. Environment
My code successfully compiled with cmake and make using default CMakeLists.txt in the following environments:
* OS: Windows10 and Ubuntu 16.4 bash on WSL of Windows 10 Fall Creators Update.
* cmake v3.5.1
* GNU Make v4.1
* gcc/g++ v5.4.0

[//]: # (Image References)
[mymodel.png]: ./mymodel.png

---
## 2. Writeup / README

### 2-1. Compilation
#### 2-1-1. CRITERIA: Your code should compile.
My code was successfully compiled without errors.

---
### 2-2. Implementation
#### 2-2-1. CRITERIA: The Model. Student describes their model in detail. This includes the state, actuators and update equations.
My model is based on the following equations. 
![mymodel.png][mymodel.png]  
I used a vehicle coordinate system instead of world coordinate system to fit the model so that 3rd order polynomial is a good approximation of the given trajectory. Because the vehicle coordinate system as a relative coodinate is refreshed at every time step based on the current vehicle location, current egovehicle positions (x, y, psi) are always fixed to zero.

#### 2-2-2. CRITERIA: Timestep Length and Elapsed Duration (N & dt). Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
I choosed N=10 and dt=0.1sec because of the following reasons.
```cpp
size_t N = 10;
double dt = 0.1;
```
- I choosed dt=0.1s because the latency of controllers is 100msec (=0.1s), and I hypothesized that it doesn't make positive effect if I choose finer resolution of dt. When I tested dt=0.05s and 0.01s, the performance became worse indeed as I expected.
- I tried N=20 larger value of N than 20, however the it didn't work because the computing latency of  `MPC::Solve()` became larger than 100msec.


#### 2-2-3. CRITERIA: Polynomial Fitting and MPC Preprocessing. A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
in main.cpp.
Waypoints are pre-processed to transform from world coodinate system to vehicle coodinate system so that the waypoints were fitted to 3rd order polynomial well.
```cpp
Eigen::VectorXd ptsx_car_coodinate(ptsx.size());
Eigen::VectorXd ptsy_car_coodinate(ptsy.size());
// Converting to vehicle coodinate
for(size_t i=0; i<ptsx.size(); i++){
  ptsx_car_coodinate[i] =  (ptsx[i]-px)*cos(psi) + (ptsy[i]-py)*sin(psi);
  ptsy_car_coodinate[i] = -(ptsx[i]-px)*sin(psi) + (ptsy[i]-py)*cos(psi);
}
auto coeffs = polyfit(ptsx_car_coodinate, ptsy_car_coodinate, 3);
```

#### 2-2-4. CRITERIA: Model Predictive Control with Latency. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

- I choosed dt=0.1s so that the discrete time step corresponds to the controller latency which is equal to 100ms.
- I measured the actual latencis of the Model Predictive Control modules by using the following timer class. As a result, the latencies were distributing around 10msec and 30msec, and the distribution was fitted in 100msec upper bound.
```cpp
class MyTime {
  std::chrono::system_clock::time_point start_time;
public:
  void tick(){
    start_time = std::chrono::system_clock::now();
  };
  void tack(string txt){
    std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast< std::chrono::milliseconds >(end_time - start_time).count();
    std::cout << txt << ": duration = " << elapsed << "msec.\n";
  };
  void tack(void){
    tack(string("Time"));
  }
  double measure(){
    std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
    return std::chrono::duration_cast< std::chrono::milliseconds >(end_time - start_time).count();
  };
};
```


---
### 2-3. Simulation
#### 2-3-1. CRITERIA: The vehicle must successfully drive a lap around the track.
The vehicle successfully drove a lap around the track. Please refer to [YouTube video](https://youtu.be/Sg6Ga40LST8).

