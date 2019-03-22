#include "PID.h"
#include <numeric>
#include <iostream>
#include <fstream>


using std::cout;
using std::endl;

/**
 * PID class.
 */
PID::PID() {}

PID::~PID() {}

void PID::Init(vector<double> K_pdi_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  K_pdi = K_pdi_;

  p_error = 0.0;
  d_error  = 0.0;
  i_error = 0.0;

  tuning_enable = false;
}

void PID::InitTuning(vector<double> K_pdi_, int max_steps_, double tolerance_) {
    // Initialize the default PID controller variables 
    Init(K_pdi_);
    
    /**
     * Initialization of tuning relevant variables
     */
    tuning_enable = true;
    step_counter = 0;
    tuning_completed = false;
    max_steps = max_steps_;
    tuning_error = 0.0;

    /**
     * Flags used in the logic of twiddle method
     */ 
    simulation_done = {false, false, false};
    value_set = {false, false, false};
    index_K = 0;
    //1
    //dp = {0.148594 , 0.201794 , 0.135085};
    //2
    //dp = {0.00328097 , 0.00364551 , 0.00199668};
    //dp = {0.5 , 0.5 , 0.5};
    //3
    dp = {0.1028183 , 0.220673 , 0.0443147};
    //dp = {1.0 , 1.0, 1.0};
    tolerance = tolerance_;
    iterations = 1;
}

void PID::ResetTuning() {
    // Reset values used to determine simulation completion
    step_counter = 0;
    tuning_completed = false;
    tuning_error = 0.0;

    // Reset values used in PID methods
    p_error = 0.0;
    d_error  = 0.0;
    i_error = 0.0;
}


void PID::UpdateError(double cte) {
    /**
     * Update PID errors based on cte.
     */
    d_error =  cte-p_error;
    p_error =  cte;
    i_error += cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  double total_error = -K_pdi[0]*p_error - K_pdi[1]*d_error -K_pdi[2] * i_error;
  return total_error; 
}

bool PID::isTuningEnable() {
    return tuning_enable;
}

string PID::runProcess(json input) {

     // j[1] is the data JSON object
    double cte = std::stod(input[1]["cte"].get<string>());
    double speed = std::stod(input[1]["speed"].get<string>());
    double angle = std::stod(input[1]["steering_angle"].get<string>());

    /**
     * Calculate steering value here, remember the steering value is
     *   [-1, 1].
     * NOTE: Feel free to play around with the throttle and speed.
     *   Maybe use another PID controller to control the speed!
     */
    UpdateError(cte);
    double steer_value = TotalError();
    
    double max_speed = 30;
    double throttle;
    // Stop acceleration when speed has reached max_speed
    if (speed < max_speed) throttle = 1.0;
    else throttle = 0.0;

    // DEBUG
    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

    // Telemetry control signal from PID
    json msgJson;
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle;
    string msg = "42[\"steer\"," + msgJson.dump() + "]";
    //cout << msg << endl;

    /**
     * Peforms calculations relevant to the tunning process when tuning_enable 
     * flag is true. Increments step_counter, ,  and finally 
     */ 
    if (tuning_enable) {
        step_counter += 1;
        // Accumulate error after max_steps/2 is surpassed
        if (step_counter >= (max_steps/2)) {
            tuning_error += cte*cte;
            //cout << "Error tuning evolution: " <<error_tuning << endl;
        }
        // Prints number of steps every 200 steps
        if (step_counter % 100 == 0)
        {
            cout << "Number of steps: " << step_counter << endl;
        }
        //set the flag tuning_completed to true and update the tuning_error when the simulation is done
        if (step_counter == max_steps) {
            tuning_completed = true;
            tuning_error = tuning_error/(double(max_steps)/2.0);
        } 
    }
    
    return msg;
}

string PID::TwiddleTunning(json input,  dual_stream& ds) {
    // Perform initial simulation until simulation_done[0] flag is true
    if (!simulation_done[0]) {
        string msg = runProcess(input);
        // When simulation complete, store tuning_error in best_error and flag simulation 0 as done
        if (tuning_completed) {
            best_error = tuning_error;
            simulation_done[0] = true;
            ds << "Iteration "<< iterations << " completed - Best error: "<< best_error <<" Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] << "} dp: {" <<dp[0] <<" , "\
                                        << dp[1] << " , " << dp[2] << "}" << endl;
            iterations += 1;
            
        }
        return msg;
    // When simulation 0 is done...
    } else {
        double dp_total = 0.0;
        for (unsigned int i = 0; i < dp.size(); ++i) dp_total += dp[i];
        // Verify that accumulator of dp values is bigger than the tolerance
        if (dp_total > tolerance) {
            // Set values and restart the simulation when flag value_set[0] is not true
            if (!value_set[0]) {
                // Increase K_pdi constant by dp at index_K
                K_pdi[index_K] += dp[index_K];
                value_set[0] = true;
                // Simulation start message
                ds << "\nIteration starts -> Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] << "} dp: {" <<dp[0] <<" , "\
                                        << dp[1] << " , " << dp[2] << "}" << endl;
                // Reset PID tuning variables
                ResetTuning();
                // Reset simulation
                string msg = "42[\"reset\",{}]";
                return msg;             
            }
            // Perform simulation 1 until simulation_done[1] flag is true
            if(!simulation_done[1]) {
                string msg = runProcess(input);
                // When simulation complete, store tuning_error in current_error and flag simulation 1 as done
                if (tuning_completed) {
                    current_error = tuning_error;
                    simulation_done[1] = true;
                }
                return msg;
            } else {
                // If current simulation had better performance than previous ones (using error metric)
                if (current_error < best_error) {
                    // Update best error value
                    best_error = current_error;
                    // Modify step of constant by 1.1
                    dp[index_K] *= 1.1;
                    
                    ds << "Iteration "<< iterations << " completed - Best error improved: "<< best_error << endl;
                    iterations += 1;

                    // Move to next constant
                    index_K += 1;
                    if (index_K == dp.size()) index_K = 0; 
                    // Restart twiddle method flags
                    simulation_done = {true, false, false};
                    value_set = {false, false};
                    
                    // Reset PID tuning variables
                    ResetTuning();
                    // Reset simulation
                    string msg = "42[\"reset\",{}]";
                    return msg;
                    
                } else {
                    // Set values and restart the simulation when flag value_set[1] is not true
                    if (!value_set[1]) {
                        // Decrease K_pdi constant by 2*dp at index_K
                        K_pdi[index_K] -= 2*dp[index_K];
                        value_set[1] =  true;
                        // best_error wasn't changed. Print message to state that fact
                        ds << "Iteration "<< iterations << " completed - Best error remains: "<< best_error << endl;
                        iterations += 1;
                        // Simulation start message
                        ds << "\nIteration starts -> Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] << "} dp: {" <<dp[0] <<" , "\
                                        << dp[1] << " , " << dp[2] << "}" << endl;

                        // Reset PID tuning variables
                        ResetTuning();
                        // Restart simulation
                        string msg = "42[\"reset\",{}]";
                        return msg;
                    }
                    // Perform simulation 2 until simulation_done[2] flag is true
                    if (!simulation_done[2]) {
                        string msg = runProcess(input);
                        // When simulation complete, store tuning_error in current_error and flag simulation 2 as done
                        if (tuning_completed) {
                            current_error = tuning_error;
                            simulation_done[2] = true;
                        }
                        return msg;
                    } else {
                        // If current simulation had better performance than previous ones (using error metric)
                        if (current_error < best_error) {
                            // Update best error value
                            best_error = current_error;
                            // Modify step of constant by 1.1
                            dp[index_K] *= 1.1;
                            // best_error was improved in this iteration. Print message to state that fact
                            ds << "Iteration "<< iterations << " completed - Best error improved: "<< best_error << endl;
                            iterations += 1;
                        } else {
                            // Increase K_pdi constant by dp at index_K
                            K_pdi[index_K] += dp[index_K];
                            // Modify step of constant by 0.9
                            dp[index_K] *= 0.9;
                            ds << "Iteration "<< iterations << " completed - Best error remains: "<< best_error << endl;
                            iterations += 1;
                        }

                        // Move to the next constant
                        index_K += 1;
                        if (index_K == dp.size()) index_K = 0;
                        // Restart twiddle method flags 
                        simulation_done = {true, false, false};
                        value_set = {false, false};

                        // Reset PID tuning variables
                        ResetTuning();
                        // Restart simulation
                        string msg = "42[\"reset\",{}]";
                        return msg;
                    }
                }
            } 
        // If accumulator of dp values is less than the tolerance
        } else {
            // Set tuning as completed by disabling tuning
            tuning_enable = false;
            
            // Tuning complete message
            ds << "Tuning Complete. Total iterations: "<< iterations<< ".\nSimulation starts -> Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] <<"}"<< endl;
            // Reset PID tuning variables                            
            ResetTuning();
            // Restart simulation
            string msg = "42[\"reset\",{}]";
            return msg;
        }
    }
}