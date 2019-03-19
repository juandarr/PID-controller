#include "PID.h"

/**
 * Complete the PID class. You may add any additional desired functions.
 */

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
    /**
     * Update PID errors based on cte.
     */
    error_sum += cte;

    p_error = -Kp * cte;
    d_error = -Kd * (cte-prev_cte);
    i_error = -Ki * (error_sum);

    prev_cte = cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  double total_error = p_error + d_error + i_error;
  return total_error;  // TODO: Add your total error calc here!
}