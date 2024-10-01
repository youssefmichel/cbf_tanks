
#include "min_jerk.h"

MinJerk::MinJerk( realtype dt, realtype t_f, Vec x0, Vec xf):

    dt_(dt),
    t_f_(t_f),
    x0_(x0),
    xf_(xf)
{

}

void MinJerk::Update(realtype t)
{
    realtype tau = t / t_f_;
    if (tau>1){
        tau = 1;
    }
    x_d_ = x0_ + ((x0_ - xf_)*((15 * (pow(tau,4))) - (6 * (pow(tau, 5))) - (10 * (pow(tau, 3)))));

}

Vec MinJerk::GetDesiredPos()
{
    return x_d_;
}
