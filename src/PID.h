#ifndef PID_H
#define PID_H

#include <vector>
#include <string>
#include "json.hpp"
#include "dual_stream.h"

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
    void Init(vector<double> K_pdi_);

    /**
     * Initialize all variables for tuning PID controller with twiddle.
     * @param (Kp_,Kd_,Ki_) The initial PID coefficients
     */
    void InitTuning(vector<double> K_pdi_, int max_steps_, double tolerance_);

    /**
     * Reset all variables relevant to the PID controller tuning
     */ 
    void ResetTuning();

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
     * Get tunning completed flag
     */
    bool isTuningEnable(); 

    /**
     * Run PID control process
     */
    string runProcess(json input); 

    /**
    * Tune the PID parameters using twiddle method
    */ 
    string TwiddleTunning(json input, dual_stream& ds);
  
  private:
    /**
     * PID Errors
     */
    double p_error;
    double d_error;
    double i_error;
    

    /**
     * PID Coefficients
     */ 
    vector<double> K_pdi;
 
    /**
     * Previous values
    */ 
    // Previous error
    double prev_cte;
    // Error accumulator
    double error_sum;

    /**
     * Tuning relevant variables
     */
    // True when InitTuning method has been used
    bool tuning_enable;
    // True when tunning is complete
    bool tuning_completed;
    // Number of steps since the beginning of the simulation
    int step_counter; 
    // Max number of steps per simulation trial in twiddle tunning
    int max_steps;
    // Tunning error when the system is in a steady state (after n inital steps)
    double tuning_error;
    // Best error in simulations
    double best_error;
    // Current error after completion of simulation
    double current_error;

    /**
     * Flags used in the logic of twiddle method
     */ 
    // Elements are true when simulations are done, otherwise false
    vector<bool> simulation_done;
    // Elements are true when values have been set, otherwise false
    vector<bool> value_set;
    // Index K iterator
    unsigned int index_K;
    // Increase/decrease step value for modification of PID constants in twiddle
    vector<double> dp;
    // Tolerance
    double tolerance;
    int iterations ;
};

#endif  // PID_H