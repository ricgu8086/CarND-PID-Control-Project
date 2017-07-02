#ifndef PID_H
#define PID_H
 
class PID
{
    public:
        /*
         * Errors
         */
        double p_error;
        double i_error;
        double d_error;
 
        double prev_cte;
        /*
         * Coefficients
         */
        double Kp;
        double Ki;
        double Kd;
 
        /*
         * Constructor
         */
        PID();
 
        /*
         * Destructor.
         */
        virtual ~PID();
 
        /*
         * Initialize PID.
         */
        void Init(double Kp, double Ki, double Kd);
        /*
         * Initialize PID. Same behaviour as above method, just change calling parameters.
         */
        void Init(std::vector<double> p);
 
        /*
         * Update the PID error variables given cross track error.
         */
        void UpdateError(double cte);
 
        /*
         * Calculate the total PID error.
         */
        double TotalError();
 
        /*
         * Twiddle algorithm for parameter tuning of the PID controller.
         * Note: this algorithm cannot run in practice because of the event-oriented nature of this code. But it is left here
         * for the sake of clarity and as a guide for integrating this algorithm in main.cpp
         */
        double Twiddle(std::vector<double> p, std::vector<double> dp, double (*run_func) (std::vector<double>), double tol=1e-2);
};
 
#endif /* PID_H */