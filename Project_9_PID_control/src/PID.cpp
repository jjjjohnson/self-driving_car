#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double KP, double KI, double KD, bool init) {
    Kp = KP;
    Ki = KI;
    Kd = KD;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    is_initialized = init;
}

void PID::UpdateError(double cte) {
    i_error += cte;
    d_error = cte - p_error;
    p_error = cte;

}

double PID::TotalError() {
    double err =  -1*(Kp*p_error + Ki*i_error + Kd*d_error);
    err = (err >1? 1:err);
    err = (err <-1? -1: err);
    return err;
}

