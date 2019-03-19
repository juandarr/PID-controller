#ifndef PID_H
#define PID_H

#include <vector>
#include <string>
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

class PID {
  public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Ki_, double Kd_, int max_steps_);

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
     * Set tunning enable flag to true/false
     */ 
    void setTunning(bool value);
    
    /**
     * Get the step counter value
     */ 
    int getStepCounter();
    
    /**
     * Get max steps value
     */ 
    int getMaxSteps();

    /**
     * Get tunning error value
     */
    double getTunningError(); 
    
    /**
     * Run PID control process
     */
    string runProcess(json input); 

    

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

    /**
     * Tunning relevant variables
     */
    bool tunning_enable;
    // Number of steps since the beginning of the simulation
    int step_counter; 
    int max_steps;
    // Tunning error when the system is in a steady state (after n inital steps)
    double error_tunning;

    
};

#endif  // PID_H