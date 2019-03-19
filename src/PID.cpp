#include "PID.h"
#include <numeric>
#include <vector>

using std::vector;

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

vector<double> PID::tunningTwiddle(double tol) {

    vector<double> p = {0.0,0.0,0.0};
    vector<double> dp = {1.0,1.0,1.0};

    // auto = auto.init()
    // x_traj, y_traj, best_error = simulator.run(auto, p) for n interations
    double best_error = 0.0;
    while (std::accumulate(dp.begin(), dp.end(), 0) > tol) {
        for (unsigned int i = 0; i < p.size(); ++i) {
            p[i] += dp[i];

            //auto = auto.init()
            //x_traj, y_traj, error = simulator.run(auto, p) for n interations
            double error = 0.0;
            if (error < best_error) {
                best_error = error;
                dp[i] *= 1.1;
            } else {
                p[i] -= 2*dp[i];
                //auto =  auto.init()
                //x_traj, y_traj, error = simulator.run(auto, p) for n interations
                double error = 0.0;
                if (error < best_error) {
                    best_error = error;
                    dp[i] *= 1.1;  
                }
                else {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }    
        }
    }
    p.push_back(best_error);
    return p; 
}