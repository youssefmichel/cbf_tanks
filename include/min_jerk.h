#ifndef MINJERK_H_
#define MINJERK_H_

#include "utility.h"


class MinJerk {
private:
        realtype t_tot_;
        realtype dt_;
        realtype t_f_;
        Vec x0_;
        Vec xf_;
        Vec x_d_;



public:
        MinJerk( realtype dt_, realtype t_f_, Vec x0_, Vec xf_);
        void Update(realtype t);
        Vec GetDesiredPos();

};

#endif
