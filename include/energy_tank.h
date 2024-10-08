#ifndef ENERGYTANK_H_
#define ENERGYTANK_H_

#include "utility.h"
#include <ros/ros.h>
#include <ros/package.h>

//Base Class For an energy_tank


class EnergyTank{

 protected:
    realtype Pow_act_ ;
    realtype Pow_max_ ;
    virtual bool Initiliaze(double dt, int activate_tank)=0;
    virtual  realtype UpdateTank(realtype Pow_des)=0 ;
    realtype dt_ ;

 public:
    EnergyTank(){}
    ~EnergyTank(){}


};

#endif
