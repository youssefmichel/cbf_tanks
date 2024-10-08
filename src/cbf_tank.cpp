
#include "cbf_tank.h"



bool CBFTank::Initiliaze(double dt,int ACTIVATE_TANK) {

    if(! ros::param::get("Pow_max",Pow_max_)) {
        ROS_WARN("Max Power Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_min",s_cbf_min_)) {
        ROS_WARN("Min Energy Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_init",s_cbf_init_)) {
        ROS_WARN("Initial Energy Param not found !!");
        return false;
    }
    if(! ros::param::get("Tankgain",k_f_)) {
        ROS_WARN("Tank gain Param not found !!");
        return false;
    }
    if(! ros::param::get("cbf_rate",cbf_rate_)) {
        ROS_WARN("cbf_rate Param not found !!");
        return false;
    }

    s_cbf_max_=s_cbf_init_ ;
    s_cbf_dot_= 0.0 ;
    s_cbf_= s_cbf_init_ ;
    alpha_=1 ;
    Pow_prev_=0 ;

    dt_=dt;

    cbf_tank_active_=false ;
    P_thresh_min_=0.001 ;
    activate_tank_= ACTIVATE_TANK ;


    ROS_INFO("CBF Tank Initialized !!");

    return true ;

}


realtype CBFTank::UpdateTank(realtype Pow_des) {

   if(activate_tank_){
    if(cbf_tank_active_) {
        P_thresh_min_=0.024;
    }
    else {
        P_thresh_min_=0.024 ;
    }

    if(abs(Pow_des) >P_thresh_min_){
        u_cbf_=compute_u_cbf(Pow_des) ;
        alpha_=abs(u_cbf_/Pow_des)  ;

        if(alpha_>1){
            alpha_ =1 ;
       }
        cbf_tank_active_=true ;

    }
    else {
        cbf_tank_active_=false ;
        u_cbf_=0 ;
    }

    }

    else {
       u_cbf_=-Pow_des ;
       alpha_=1 ;

    }

    s_cbf_ddot_= k_f_*(u_cbf_ - s_cbf_dot_) ;
    s_cbf_dot_ = s_cbf_dot_ + dt_ * s_cbf_ddot_ ;
    s_cbf_ = s_cbf_ + dt_ * s_cbf_dot_ ;
    return alpha_;


}

realtype CBFTank::compute_u_cbf(realtype Pow_des) {

    int_t nWSR = 100;


    realtype lambda= 0 ;



    real_t lb[1] = { -100};
    real_t ub[1] = { 100};
    real_t H[1] = { 1.0 + lambda/dt_ };
    real_t A[2*1] = {k_f_,k_f_};
    real_t f[1] = {   Pow_des - (lambda*Pow_prev_)/dt_ };
    realtype tmp_lba_1=-cbf_rate_*(s_cbf_dot_ -Pow_max_) + k_f_ * s_cbf_dot_;
    realtype tmp_lba_2=(-2*cbf_rate_+ k_f_)*s_cbf_dot_ - (pow(cbf_rate_,2) * (s_cbf_-s_cbf_min_));

    real_t lbA[2] = {  tmp_lba_1,tmp_lba_2};
    real_t ubA[1] = { 500 };

    QProblem CBF(1,2) ;
    CBF.setPrintLevel(PL_NONE) ;
    CBF.init( H,f,A,lb,ub,lbA,NULL, nWSR  );

    real_t xOpt[1];
    CBF.getPrimalSolution( xOpt );


    Pow_prev_= xOpt[0] ;


    return xOpt[0] ;

}

void CBFTank::set_Pow_limt(realtype P_max){

    Pow_max_=P_max  ;

}

realtype CBFTank::smooth_sigma_map(realtype P, realtype P_min, realtype P_max,realtype k_f){



        if (P <= P_min) {
            alpha_ = 1;
        }
        else if (P >= P_max) {
            alpha_ = 0;
        }
        else {
            alpha_ = 0.5 + 0.5 * cos(PI * (P - P_min) / (P_max - P_min));
        }

          return k_f*  alpha_ ;
}


realtype CBFTank::get_s_cbf() const{
    return s_cbf_ ;
}

realtype CBFTank::get_s_cbf_dot() const{
    return s_cbf_dot_ ;
}


