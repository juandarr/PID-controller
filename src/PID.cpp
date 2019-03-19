#include "PID.h"

/**
 * PID class.
 */
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, int max_steps_=400) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  prev_cte = 0.0;
  error_sum = 0.0;

  p_error = 0.0;
  i_error = 0.0;
  d_error  = 0.0;
  
  step_counter = 0;
  error_tunning = 0.0;
  max_steps = max_steps_;
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
  return total_error; 
}

void PID::setTunning(bool value) {
    tunning_enable = value;
}

int PID::getStepCounter() {
    return step_counter;
}


int PID::getMaxSteps() {
    return max_steps;
}

double PID::getTunningError() {
    return error_tunning;
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
    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

    // Telemetry control signal from PID
    json msgJson;
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle;
    string msg = "42[\"steer\"," + msgJson.dump() + "]";
    std::cout << msg << std::endl;

    step_counter += 1;

    if (tunning_enable) {
        if (step_counter >= (max_steps/2)) {
            error_tunning += cte*cte;
        }
    }
    

    return msg;
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
                //auto =  auto.init() --Reset simulator
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