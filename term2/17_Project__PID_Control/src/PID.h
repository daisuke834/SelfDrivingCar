#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double prev_cte;
  double steer;
  double throttle;

  long N;
  long step_index;
  double total_error;
  double max_steer;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed);

  /*
  * Get a steering angle.
  */
  double GetSteer();

  /*
  * Get a throttle.
  */
  double GetThrottle();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Return the current step index.
  */
  long GetIndex();

  long GetEvaluationIndex();
};

#endif /* PID_H */
