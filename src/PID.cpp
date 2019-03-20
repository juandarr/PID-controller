#include "PID.h"
#include <numeric>
#include <iostream>

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
  
  prev_cte = 0.0;
  error_sum = 0.0;

  p_error = 0.0;
  d_error  = 0.0;
  i_error = 0.0;

  tuning_enable = false;
}

void PID::TuningInit(vector<double> K_pdi_, int max_steps_) {
    // Initialize the default PID controller variables 
    Init(K_pdi_);
    
    /**
     * Initialization of tuning relevant variables
     */
    tuning_enable = true;
    step_counter = 0;
    tuning_completed = false;
    max_steps = max_steps_;
    error_tuning = 0.0;

    /**
     * Flags used in the logic of twiddle method
     */ 
    simulation_done = {false, false, false};
    value_set = {false, false, false};
    index_K = 0;
    //dp = {0.016747 , 1.06572 , 0.000249128};
    //dp = {0.05, 3.5, 0.001};
    dp = {1.0 , 1.0, 1.0};
}

void PID::TuningReset() {
    // Reset values used to determine simulation completion
    step_counter = 0;
    tuning_completed = false;
    error_tuning = 0.0;

    // Reset values used in PID methods
    prev_cte = 0.0;
    error_sum = 0.0;

    p_error = 0.0;
    d_error  = 0.0;
    i_error = 0.0;
}



void PID::UpdateError(double cte) {
    /**
     * Update PID errors based on cte.
     */
    error_sum += cte;

    p_error = -K_pdi[0] * cte;
    d_error = -K_pdi[1] * (cte-prev_cte);
    i_error = -K_pdi[2] * (error_sum);
    
    
    prev_cte = cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  double total_error = p_error + d_error + i_error;
  return total_error; 
}

int PID::getStepCounter() {
    return step_counter;
}

int PID::getMaxSteps() {
    return max_steps;
}

double PID::getTuningError() {
    return error_tuning;
}

vector<double> PID::getPIDValues() {
    return K_pdi;
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
    if (speed < max_speed) throttle = 0.5;
    else throttle = 0.0;

    // DEBUG
    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

    // Telemetry control signal from PID
    json msgJson;
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle;
    string msg = "42[\"steer\"," + msgJson.dump() + "]";
    //cout << msg << endl;

    if (tuning_enable) {
        step_counter += 1;
        if (step_counter >= (max_steps/2)) {
            error_tuning += cte*cte;
            //cout << "Error tuning evolution: " <<error_tuning << endl;
        }
        if (step_counter % 200 == 0)
        {
            cout << "Number of steps: " << step_counter << endl;
        }
        if (step_counter == max_steps) {
            tuning_completed = true;
            error_tuning = error_tuning/(double(max_steps)/2.0);
        } 
    }
    
    return msg;
}

string PID::TwiddleTunning(json input, double tolerance) {
    if (!simulation_done[0]) {
        string msg = runProcess(input);
        if (tuning_completed) {
            best_error = error_tuning;
            simulation_done[0] = true;
            cout << "First iteration completed - Best error: "<< best_error <<" Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] << "} dp: {" <<dp[0] <<" , "\
                                        << dp[1] << " , " << dp[2] << "}"<< endl;
        }
        return msg;
    } else {
        double dp_total = 0.0;
        for (unsigned int i = 0; i < dp.size(); ++i) dp_total += dp[i];
        if (dp_total > tolerance) {
            if (!value_set[0]) {
                K_pdi[index_K] += dp[index_K];
                value_set[0] = true;
                
                cout << "\nIteration starts -> Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] << "} dp: {" <<dp[0] <<" , "\
                                        << dp[1] << " , " << dp[2] << "}"<< endl;

                TuningReset();
                string msg = "42[\"reset\",{}]";
                return msg;             
            }
            if(!simulation_done[1]) {
                string msg = runProcess(input);
                if (tuning_completed) {
                    current_error = error_tuning;
                    simulation_done[1] = true;
                }
                return msg;
            } else {
                if (current_error < best_error) {
                    best_error = current_error;
                    dp[index_K] *= 1.1;
                    
                    cout << "Iteration completed - Best error improved: "<< best_error << endl;

                    index_K += 1;
                    if (index_K == dp.size()) index_K = 0; 
                    simulation_done = {true, false, false};
                    value_set = {false, false, false};
                    
                    TuningReset();
                    string msg = "42[\"reset\",{}]";
                    return msg;
                    
                } else {
                    if (!value_set[1]) {
                        K_pdi[index_K] -= 2*dp[index_K];
                        value_set[1] =  true;
                        cout << "Iteration completed - Best error remains: "<< best_error << endl;
                        
                        cout << "\nIteration starts -> Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] << "} dp: {" <<dp[0] <<" , "\
                                        << dp[1] << " , " << dp[2] << "}"<< endl;
                        TuningReset();
                        string msg = "42[\"reset\",{}]";
                        return msg;
                    }
                    if (!simulation_done[2]) {
                        string msg = runProcess(input);
                        if (tuning_completed) {
                            current_error = error_tuning;
                            simulation_done[2] = true;
                        }
                        return msg;
                    } else {
                        if (current_error < best_error) {
                            best_error = current_error;
                            dp[index_K] *= 1.1;
                            cout << "Iteration completed - Best error improved: "<< best_error << endl;
                        } else {
                            K_pdi[index_K] += dp[index_K];
                            dp[index_K] *= 0.9;
                            cout << "Iteration completed - Best error remains: "<< best_error << endl;
                        }

                        index_K += 1;
                        if (index_K == dp.size()) index_K = 0; 
                        simulation_done = {true, false, false};
                        value_set = {false, false, false};

                        TuningReset();
                        string msg = "42[\"reset\",{}]";
                        return msg;
                    }
                }
            } 
        } else {
            tuning_enable = false;
            
            cout << "Tuning Complete.\nSimulation starts -> Kpdi: {"<< K_pdi[0] <<" , "\
                                        << K_pdi[1] << " , " << K_pdi[2] <<"}"<< endl;
            TuningReset();
            string msg = "42[\"reset\",{}]";
            return msg;
        }
    }
}