#include "PID.h"
#include <cmath>
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    twiddle = false;
    error = 0;
    best_error = 9999999;
    dp = {Kp*.1,Ki*.1,Kd*.1};
    p =  {Kp,Ki,Kd};
    i = 0;
    check = true;
    step = 0;
}
//TODO: Skip initial steps before twiddling
void PID::UpdateError(double cte) {
    //new cte - cte(t-1)
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    /*Note: twiddle affected heavly by simulator/computer performance*/
	/*dont run until the 1000th step, then accumulate over 500 steps and run every 500*/
    /*accumulate the error over some time then twiddle*/
    if(twiddle && (step % 500 == 0) && step > 999)
    {
        cout << "PRE: " << Kp << "---" << Ki << "---" << Kd << endl;
        if(error < best_error)
        {
            best_error = error;
            dp[i] *= 1.1;
            check = true;
            //cout << "IF 1" << endl;
        }
        else
        {
            p[i] -= 2 * dp[i];
            check = false;
            //cout << "ELSE 1" << endl;
        }
        if(!check)
        {
            if(error < best_error)
            {
                best_error = error;
                dp[i] *= 1.1;
                //cout << "IF 2" << endl;
            }
            else
            {
                p[i] += dp[i];
                dp[i] *=.9;
                //cout << "ELSE 2" << endl;
            }
            check = true;
        }

        i = (i + 1) % 3;
        this->Kp = p[0];
        this->Ki = p[1];
        this->Kd = p[2];
        cout << "POST: " << Kp << "---" << Ki << "---" << Kd << endl;
        error = 0;
    }
    else
    	error += fabs(cte);

    step++;
}

double PID::TotalError() {
    //-tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
    double t_error = (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);

    return t_error;
}

