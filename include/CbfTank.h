#ifndef CBFTANK_H_
#define CBFTANK_H_



#include "Utility.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "quatUtils.h"
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

class cbf_tank{

private:

realtype s_cbf_ddot ;
realtype s_cbf_dot ;
realtype s_cbf ;


realtype Pow_act ;
realtype u_cbf ;
realtype Pow_max ;
realtype s_cbf_min ;
realtype s_cbf_max ;
realtype s_cbf_init ;
realtype k_f ;
realtype cbf_rate ;
realtype alpha ;
realtype Pow_prev ;
realtype Time_tot ;
realtype P_thresh_min ;
bool cbf_tank_active ;

// QP Variables
//real_t H_qp[1] ;
//real_t f_qp[1] ;
//real_t A_qp[1] ;
//real_t lbA_qp[1] ;
//real_t lb_qp[1];
//real_t ub_qp[1] ;

int_t n=1 ;
QProblem CBF;
int ACTIVATE_TANK ;


realtype dt ;

public:
    cbf_tank() ;
    bool Initiliaze(double dt_ ,int ACTIVATE_TANK_);
    realtype update_tank(realtype Pow_des) ;
    realtype compute_u_cbf(realtype Pow_des) ;
    realtype get_s_cbf() ;
    realtype get_s_cbf_dot() ;
    void set_Pow_limt(realtype P_max) ;
    realtype smooth_sigma_map(realtype P, realtype P_min, realtype P_max,realtype k_f) ;


};




#endif

