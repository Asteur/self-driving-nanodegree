#ifndef PID_H
#define PID_H
#include <vector>

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
  std::vector<double> K;

  std::vector<double> dp;

  //terminating criteria
  double tol;
  //best error
  double best_error;

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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  double Twiddle(bool twiddle,double cte) ;

  //0 initial state,1 previous increment state ,2 previous decrement state
  int twiddleState=0;

  //parameter index
  int index=0;

};

#endif /* PID_H */
