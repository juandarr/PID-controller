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
    void Init(vector<double> K_pdi_);

    /**
     * Initialize all variables for tuning PID controller with twiddle.
     * @param (Kp_,Kd_,Ki_) The initial PID coefficients
     */
    void TuningInit(vector<double> K_pdi_, int max_steps_);

    /**
     * Reset all variables relevant to the PID controller tuning
     */ 
    void TuningReset();

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
    double getTuningError(); 
    
    /**
     * Get tunning completed flag
     */
    bool isTuningEnable(); 

    /**
     * Get pid values
     */
    vector<double> getPIDValues(); 

    /**
     * Run PID control process
     */
    string runProcess(json input); 

    /**
    * Tune the PID parameters using twiddle method
    */ 
    string TwiddleTunning(json input, double tolerance);
  
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
     * Previous values storage
    */ 
    double prev_cte;
    double error_sum;

    /**
     * Tunning relevant variables
     */
    bool tuning_enable;
    // True when tunning is complete
    bool tuning_completed;
    // Number of steps since the beginning of the simulation
    int step_counter; 
    // Max number of steps per simulation trial in twiddle tunning
    int max_steps;
    // Tunning error when the system is in a steady state (after n inital steps)
    double error_tuning;
    // Used as temporal error variable in twiddle method
    double best_error;
    double current_error;

    /**
     * Flags used in the logic of twiddle method
     */ 
    vector<bool> simulation_done;
    vector<bool> value_set;
    unsigned int index_K;
    // Differential variable used to change PID constants in twiddle
    vector<double> dp;
};

#endif  // PID_H