#ifndef FIRSTORDERTANK_H_
#define FIRSTORDERTANK_H_
#pragma once
#endif

#include "Utility.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "quatUtils.h"
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

class firstorder_tank{

protected:


realtype E_tank ;

realtype Pow_act ;
realtype Pow_max ;
realtype E_tank_min ;
realtype E_tank_max ;
realtype E_tank_init ;


realtype alpha ;
realtype beta ;

realtype dt ;

public:
    firstorder_tank() ;
    bool Initiliaze(double dt_);
    realtype update_tank(realtype Pow_des) ;
    realtype get_tank_state() ;

};

class firstorder_tank_cbf: public firstorder_tank {

private:

    realtype x_tank ;
    realtype y_tank ;
    void set_xy_tank() ;
    realtype compute_u_cbf_1st(realtype Pow_des) ;
    realtype Pow_opt ;


public:
    realtype update_tank(realtype Pow_des) ;
    bool Initiliaze(double dt_) ;
    realtype Get_Pow_opt() ;
    realtype get_tank_state() ;

} ;






