#ifndef PID_H
#define PID_H

#include <vector>

using std::vector;

class PID {
 public:
  /**
   * Constructor
   */
  PID() : prev_cte(0.0), error_sum(0.0) {};

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Tune the PID parameters using twiddle method
   */ 
  vector<double> tunningTwiddle(double tol);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Previous values storage
  */ 
  double prev_cte;
  double error_sum;
};

#endif  // PID_H