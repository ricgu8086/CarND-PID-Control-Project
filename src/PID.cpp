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

    cout << "PID controller initializing with parameters: " << endl 
        << "Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << endl << "--------" << endl;
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
 
    p = {0.436437, 0.0467251, 7.44939};
    dp = {0.005, 0.005, 0.005};

    // History of values
    /*
    p = {0, 0, 0};
    dp = {1, 1, 1};
    RESULTS: when the first parameter is increased it overshoots too much
    p = {0, 0, 0};
    dp = {0.5, 0.5, 0.5};
    RESULTS: when the first parameter takes negative values behaves bad
    p = {0.5, 0.5, 0.5};
    dp = {0.3, 0.3, 0.3};
    RESULTS: Kp seems stable near 0.37, I will reduce all the intervals, despite it will converge much slower, at least the car will be able to drive autonomous more time.
    p = {0.37, 0.5, 0.5};
    dp = {0.15, 0.15, 0.15};
    RESULTS: It works better, but everytime it touches Kp, it start overshooting and break everything. Solution: tiny dp for Kp and let the other parameters converge
    p = {0.37135, 0.6365, 0.94715};
    dp = {0.02, 0.15, 0.15};
    RESULTS: Better, but I don't want to touch Kp anymore until reaching better values for the others
    p = {0.340359, 1.31219, 1.48609};
    dp = {0.0, 0.15, 0.15};
    RESULTS: Nice. Seems that a good value for Kd and Ki are much higher and their's dp is limiting convergence. Sol: let's double them.
    p = {0.340359, 2.22758, 2.40496};
    dp = {0.0, 0.3, 0.3};
    RESULTS: Ki and Kp continue increasing, so it was a good idea. However, a high Ki seems a bit odd. I will do as in the lessons: firts Kp, next Kd and then Ki.
    p = {0.340359, 0, 3.97007};
    dp = {0.0, 0.0, 0.3};
    RESULTS: It was an awesome idea!! Now the car is able to do a full lap without going outside the lane. Now it's time to allow a bit movement in Kp.
    p = {0.340359, 0, 6.46975};
    dp = {0.1, 0.0, 0.1};
    RESULTS: Even better. Now it is very stable. Time for Ki to do its magic.
    p = {0.396611, 0, 7.37227};
    dp = {0.02, 0.1, 0.01};
    RESULTS: When it's varying Ki it breaks. dp is very high for it, need to reduce.
    p = {0.396611, 0, 7.37227};
    dp = {0.02, 0.05, 0.01};
    RESULTS: still very sensitive
    p = {0.396611, 0, 7.37227};
    dp = {0.02, 0.01, 0.01};
    RESULTS: Done. Final parameters -> p = {0.436437, 0.0467251, 7.44939};
    */
 
    warmup_iterations = 50;
    total_iterations = 2*warmup_iterations;
    curr_iterations = 0;
    tol = 1e-3;
    best_error = 0;
    error = 0;
    accumulated_error = 0;
    sum_dp = 0;
    i = 0;
 
    curr_state = State::BEGINNING;
    Init(p);
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
        cout << "Cycle completed" << endl;

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
            cout << endl;
            cout << "Parameter tuning accomplished" << endl;
            cout << "Params: " << p[0] << ", " << p[1] << ", " << p[2] << endl;
            cout << "Best error: " << best_error;
 
            return true; // is_finished = true
        }
    }

    return false; // is_finished = false
}