#include <iostream>
#include <vector>
  
#include "PID.h"
  
using namespace std;
  
/*
 * : Complete the PID class.
 */
  
PID::PID()
{
    is_twiddle_on = false; // Cannot be in Init 'cause it is called many times in Twiddle algorithm
}
  
PID::~PID()
{
}
  
void PID::Init(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
  
    p_error = 0;
    i_error = 0;
    d_error = 0;
  
    prev_cte = 0;
}
  
void PID::Init(vector<double> p)
{
    PID::Init(p[0], p[1], p[2]);
}
  
void PID::UpdateError(double cte)
{
    p_error = cte;
    i_error += cte;
    d_error = cte - prev_cte;
  
    prev_cte = cte;
}
  
double PID::TotalError()
{
    double steer = - Kp*p_error \
                   - Ki*i_error \
                   - Kd*d_error;
    return steer;
}
  
  
double PID::Twiddle(vector<double> p, vector<double> dp, double (*run_func) (vector<double>), double tol)
{
    double best_error = run_func(p); // state::BEGINNING
    double error = 0;
    double sum_dp = 0;
  
    for(auto n: dp)
        sum_dp += n;
  
    while (sum_dp > tol)
    {
        for(unsigned int i=0; i<p.size(); i++)
        {
            p[i] += dp[i];
            // Reset controller state
            error = run_func(p);  // state::BLOCK1
  
            if(error < best_error)
            {
                best_error = error;
                dp[i] *= 1.1;
            }
            else
            {
                p[i] -= 2*dp[i];
                // Reset controller state
                error = run_func(p);  // state::BLOCK2
  
                if(error < best_error)
                {
                    best_error = error;
                    dp[i] *= 1.1;
                }
                else
                {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
  
        }
  
        for(auto n: dp)
            sum_dp += n;
    }
  
    return best_error; // state::FINISHED
}
 
void PID::Twiddle_On()
{
    is_twiddle_on = true;
 
    p = {0, 0, 0};
    dp = {1, 1, 1};
 
    warmup_iterations = 100;
    total_iterations = 2*warmup_iterations;
    curr_iterations = 0;
    tol = 1e-2;
    best_error = 0;
    error = 0;
    accumulated_error = 0;
    sum_dp = 0;
    i = 0;
 
    curr_state = State::BEGINNING;
}
 
bool PID::Twiddle_event_oriented(double cte)
{
    curr_iterations += 1;
 
    if(curr_iterations >= warmup_iterations)
    {
        accumulated_error += cte*cte;
    }
 
    if(curr_iterations == total_iterations) // A cycle is completed
    {
        curr_iterations = 0;
        error = accumulated_error/(double)warmup_iterations;
        accumulated_error = 0;
 
        switch(curr_state)
        {
            case State::BEGINNING :
                best_error = error;
 
                sum_dp = 0;
 
                for(auto n: dp)
                    sum_dp += n;
 
                if (sum_dp > tol)
                {
                    i = 0;
                    p[i] += dp[i];
                    Init(p); // Reset controller state
                    curr_state = State::BLOCK1;
                }
 
                else
                    curr_state = State::FINISHED;
 
                break;
 
            case State::BLOCK1 :
 
                if (error < best_error)
                {
                    best_error = error;
                    dp[i] *= 1.1;
 
                    i += 1;
 
                    if (i == p.size())
                    {
                        sum_dp = 0;
 
                        for(auto n: dp)
                                sum_dp += n;
 
                        if (sum_dp > tol)
                        {
                            i = 0;
                            p[i] += dp[i];
                            Init(p); // Reset controller state
                            curr_state = State::BLOCK1;
                        }
                        else
                            curr_state = State::FINISHED;
                    }
                }
                else
                {
                    p[i] -= 2*dp[i];
                    Init(p); // Reset controller state
                    curr_state = State::BLOCK2;
                }
 
                break;
 
            case State::BLOCK2 :
 
                if(error < best_error)
                {
                    best_error = error;
                    dp[i] *= 1.1;
                }
                else
                {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
 
                i += 1;
 
                if (i == p.size())
                {
                    sum_dp = 0;
 
                    for(auto n: dp)
                            sum_dp += n;
 
                    if (sum_dp > tol)
                    {
                        i = 0;
                        p[i] += dp[i];
                        Init(p); // Reset controller state
                        curr_state = State::BLOCK1;
                    }
                    else
                        curr_state = State::FINISHED;
                }
 
                break;

            case State::FINISHED : // We should not reach here. This is just to turn off a warning
                break;
        }
 
        if (curr_state == State::FINISHED)
        {
            cout << "Parameter tuning accomplished" << endl;
            cout << "Params: " << p[0] << ", " << p[1] << ", " << p[2] << endl;
            cout << "Best error: " << best_error;
 
            return true; // is_finished = true
        }
    }

    return false; // is_finished = false
}