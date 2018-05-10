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
    best_error = 999999;
    dp = {0.1*Kp,0.1*Kd,0.1*Ki};
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

    double error = cte;
    if(step == 100 && twiddle)
    {
        cout << "-------Step: " << step << endl;

        if(error < best_error)
        {
            best_error = error;
            dp[i] *= 1.1;
            check = true;
        }
        else
        {
            p[i] -= 2 * dp[i];
            check = false; 
        }
        if(!check)
        {
            if(error < best_error)
            {
                best_error = error;
                dp[i] *= 1.1;
            }
            else
            {
                p[i] += dp[i];
                dp[i] *=.9;
            }
            check = true;
        }

        i = (i + 1) % 3;
        this->Kp += p[0];
        this->Kd += p[2];
        this->Ki += p[1];
        cout << Kp << "---" << Ki << "---" << Kd << endl;
        step = 0;
    }

    step++;
}

double PID::TotalError() {
    //-tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
    double t_error = (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);

    return t_error;
}

