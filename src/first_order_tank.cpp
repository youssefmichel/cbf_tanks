
#include "first_order_tank.h"





bool FirstOrderTank::Initiliaze(double dt,int activate_tank) {
    if(! ros::param::get("Pow_max",Pow_max_)) {
        ROS_WARN("Max Power Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_min",E_tank_min_)) {
        ROS_WARN("Min Energy Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_init",E_tank_init_)) {
        ROS_WARN("Initial Energy Param not found !!");
        return false;
    }



    E_tank_max_=E_tank_init_ ;
    E_tank_= E_tank_init_ ;
    alpha_=1 ;
    beta_=1 ;
    dt_=dt;



    ROS_INFO("First Tank Initialized !!");

    return true ;

}


realtype FirstOrderTank::UpdateTank(realtype Pow_des) {

    if(Pow_des>0 & E_tank_<E_tank_min_){
        alpha_ =0 ;
    }
    else {
        alpha_ =1 ;
    }

    if(Pow_des<0 & E_tank_>E_tank_max_) {
       beta_= 0 ;
    }
    else {
        beta_ =1 ;
    }

    if (Pow_des >0) {
        E_tank_=E_tank_ - dt_*alpha_*Pow_des ;
    }
    else {
         E_tank_=E_tank_ - dt_*beta_*Pow_des ;
    }

    return alpha_ ;

}


realtype FirstOrderTank::GetTankState() const{
    return E_tank_ ;
}





bool FirstOrderTankCBF::Initiliaze(double dt,int activate_tank) {

    FirstOrderTank::Initiliaze(dt,activate_tank) ;
    set_xy_tank();


    return true ;


}


realtype FirstOrderTankCBF::Get_Pow_opt() const{
    return Pow_opt_ ;
}

realtype FirstOrderTankCBF::UpdateTank(realtype Pow_des) {



     Pow_opt_=compute_u_cbf_1st(-Pow_des) ;
     E_tank_=E_tank_ + dt_*Pow_opt_ ;
     set_xy_tank();

     if(Pow_des >0.01 ) {
     alpha_ = abs(Pow_opt_/Pow_des) ;
            }

    return alpha_ ;

}

realtype FirstOrderTankCBF::GetTankState() const {

return 0.5*x_tank_*x_tank_ ;
}

void FirstOrderTankCBF::set_xy_tank() {

    x_tank_ = std::sqrt(2*E_tank_) ;
    y_tank_ =x_tank_ ;
}

realtype FirstOrderTankCBF::compute_u_cbf_1st(realtype u_t_des) {


    int_t nWSR = 100;
    real_t lb[1] = { -100};
    real_t ub[1] = { 100};
    real_t H[1] = { 1.0  };
    real_t A[1] = {1}; //x_tank
    real_t f[1] = {  -u_t_des };
    realtype tmp_lba_1= -2.5* ( 0.5 * x_tank_*x_tank_ - E_tank_min_)   ;

    real_t lbA[1] = {  tmp_lba_1};
    real_t ubA[1] = { 500 };

    QProblem CBF_(1,1) ;
    CBF_.setPrintLevel(PL_NONE) ;
    CBF_.init( H,f,A,lb,ub,lbA,NULL, nWSR  );

    real_t xOpt[1];
    CBF_.getPrimalSolution( xOpt );


    return xOpt[0] ;
}

