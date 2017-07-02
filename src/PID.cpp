#include <iostream>
#include <vector>
 
#include "PID.h"
 
using namespace std;
 
/*
 * : Complete the PID class.
 */
 
PID::PID()
{
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