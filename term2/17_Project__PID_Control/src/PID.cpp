#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  prev_cte=-1.0;
  i_error =0.0;
  steer=0.0;
  step_index=0;
  total_error=0.0;
  N=500;
  max_steer = 1.0;
  throttle = 0.3;
}

void PID::UpdateError(double cte, double speed) {
  if(prev_cte<0.0){
    prev_cte=cte;
  }
  p_error = cte;
  //p_error = cte*cte*cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
  i_error += cte;

  steer = -Kp*p_error - Kd*d_error - Ki*i_error;
  steer = (steer> max_steer ? max_steer : steer);
  steer = (steer<-max_steer ? -max_steer : steer);

  throttle = 0.3-1.0*steer*steer;
  throttle = (throttle> 0.0 ? throttle : 0.0);
  throttle = ( (throttle< 0.001 && speed> 5.0) ? throttle : 0.3);

  /*
  if(abs(steer)>0.8){
    throttle=0.0;
  }
  else{
    throttle=0.3;
  }
  */

  step_index+=1;
  if(step_index>N and step_index<GetEvaluationIndex())total_error+=cte*cte;
}

double PID::GetSteer() {
  return this->steer;
}

double PID::GetThrottle(){
  return this->throttle;
}

double PID::TotalError() {
  if(step_index>=GetEvaluationIndex())return total_error;
  return -1;
}

long PID::GetIndex() {
  return step_index;
}

long PID::GetEvaluationIndex() {
  return 2*N;
}
