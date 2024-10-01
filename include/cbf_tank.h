#ifndef CBFTANK_H_
#define CBFTANK_H_



#include "energy_tank.h"
#include "quatUtils.h"
#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

class CBFTank:public EnergyTank{

private:

realtype s_cbf_ddot_ ;
realtype s_cbf_dot_ ;
realtype s_cbf_ ;



realtype u_cbf_ ;

realtype s_cbf_min_ ;
realtype s_cbf_max_ ;
realtype s_cbf_init_ ;
realtype k_f_ ;
realtype cbf_rate_;
realtype alpha_ ;
realtype Pow_prev_ ;
realtype P_thresh_min_ ;
bool cbf_tank_active_ ;

// QP Variables
//real_t H_qp[1] ;
//real_t f_qp[1] ;
//real_t A_qp[1] ;
//real_t lbA_qp[1] ;
//real_t lb_qp[1];
//real_t ub_qp[1] ;


int activate_tank_ ;




public:

    bool Initiliaze(double dt_ ,int ACTIVATE_TANK) override;
    realtype UpdateTank(realtype Pow_des) override ;
    realtype compute_u_cbf(realtype Pow_des) ;
    realtype get_s_cbf() ;
    realtype get_s_cbf_dot() ;
    void set_Pow_limt(realtype P_max) ;
    realtype smooth_sigma_map(realtype P, realtype P_min, realtype P_max,realtype k_f) ;


};




#endif

