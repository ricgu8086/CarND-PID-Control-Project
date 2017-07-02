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
         * Twiddle variables
         */
        int warmup_iterations;
        int total_iterations;
        int curr_iterations;
        double tol;
 
        double best_error;
        double error;
        double accumulated_error;
        double sum_dp;
        std::vector<double> p;
        std::vector<double> dp;
        unsigned int i;
        bool is_twiddle_on;
        enum class State {BEGINNING, BLOCK1, BLOCK2, FINISHED}; // The meaning of this enum is explained in PID.cpp in PID::Twiddle
        State curr_state;

 
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
         * for the sake of clarity and as a guide for adapting the original algorithm.
         */
        double Twiddle(std::vector<double> p, std::vector<double> dp, double (*run_func) (std::vector<double>), double tol=1e-2);
 
 
        /*
         * Activate parameter tunning mode
         */
        void Twiddle_On();
 
        /*
         * Twiddle algorithm adapted to event-oriented programming. Should have the same behaviour as the Twiddle algorithm defined above.
         */
        bool Twiddle_event_oriented(double cte);
 
};
  
#endif /* PID_H */