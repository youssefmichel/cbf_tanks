#ifndef FIRSTORDERTANK_H_
#define FIRSTORDERTANK_H_
#pragma once
#endif

#include "utility.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "quatUtils.h"
#include <qpOASES.hpp>
#include "energy_tank.h"

USING_NAMESPACE_QPOASES

class FirstOrderTank:public EnergyTank{

protected:


realtype E_tank_ ;
realtype E_tank_min_ ;
realtype E_tank_max_ ;
realtype E_tank_init_ ;
realtype alpha_ ;
realtype beta_ ;


public:


    realtype GetTankState() const ;
    realtype UpdateTank(realtype Pow_des) override;
    bool Initiliaze(double dt,int activate_tank) override ;

};

class FirstOrderTankCBF: public FirstOrderTank {

private:

    realtype x_tank_ ;
    realtype y_tank_ ;
    void set_xy_tank() ;
    realtype compute_u_cbf_1st(realtype Pow_des) ;
    realtype Pow_opt_ ;


public:
    realtype UpdateTank(realtype Pow_des) override;
    bool Initiliaze(double dt,int activate_tank) override ;
    realtype Get_Pow_opt() const;
    realtype GetTankState() const;


} ;






