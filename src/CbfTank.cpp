
#include "CbfTank.h"


cbf_tank::cbf_tank(){

}

bool cbf_tank::Initiliaze(double dt_,int ACTIVATE_TANK_) {
    if(! ros::param::get("Pow_max",Pow_max)) {
        ROS_WARN("Max Power Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_min",s_cbf_min)) {
        ROS_WARN("Min Energy Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_init",s_cbf_init)) {
        ROS_WARN("Initial Energy Param not found !!");
        return false;
    }
    if(! ros::param::get("Tankgain",k_f)) {
        ROS_WARN("Tank gain Param not found !!");
        return false;
    }
    if(! ros::param::get("cbf_rate",cbf_rate)) {
        ROS_WARN("cbf_rate Param not found !!");
        return false;
    }

    s_cbf_max=s_cbf_init ;
    s_cbf_dot= 0.0 ;
    s_cbf= s_cbf_init ;
    alpha=1 ;
    Pow_prev=0 ;

    dt=dt_;
    Time_tot=0 ;
    cbf_tank_active=false ;
    P_thresh_min=0.001 ;
    ACTIVATE_TANK= ACTIVATE_TANK_ ;


    ROS_INFO("Tank Initialized properly");

    return true ;

}


realtype cbf_tank::update_tank(realtype Pow_des) {

   if(ACTIVATE_TANK){
    if(cbf_tank_active) {
        P_thresh_min=0.024;
    }
    else {
        P_thresh_min=0.024 ;
    }

    if(abs(Pow_des) >P_thresh_min){
        u_cbf=compute_u_cbf(Pow_des) ;
        alpha=abs(u_cbf/Pow_des)  ;

        if(alpha>1){
            alpha =1 ;
       }
        cbf_tank_active=true ;

    }
    else {
        cbf_tank_active=false ;
        u_cbf=0 ;
    }

    }

    else {
       u_cbf=-Pow_des ;
       alpha=1 ;

    }

    s_cbf_ddot= k_f*(u_cbf - s_cbf_dot) ;
    s_cbf_dot = s_cbf_dot + dt * s_cbf_ddot ;
    s_cbf = s_cbf + dt * s_cbf_dot ;
    return alpha;


}

realtype cbf_tank::compute_u_cbf(realtype Pow_des) {

    int_t nWSR = 100;


    realtype lambda= 0 ;

    if(Time_tot>1.5){
    // lambda=smooth_sigma_map( Pow_des, 0.01, 0.03, 10) ;
       //  lambda=3 ;
    }

    real_t lb[1] = { -100};
    real_t ub[1] = { 100};
    real_t H[1] = { 1.0 + lambda/dt };
    real_t A[2*1] = {k_f,k_f};
    real_t f[1] = {   Pow_des - (lambda*Pow_prev)/dt };
    realtype tmp_lba_1=-cbf_rate*(s_cbf_dot -Pow_max) + k_f * s_cbf_dot;
    realtype tmp_lba_2=(-2*cbf_rate+ k_f)*s_cbf_dot - (pow(cbf_rate,2) * (s_cbf-s_cbf_min));

    real_t lbA[2] = {  tmp_lba_1,tmp_lba_2};
    real_t ubA[1] = { 500 };

    QProblem CBF_(1,2) ;
    CBF_.setPrintLevel(PL_NONE) ;
    CBF_.init( H,f,A,lb,ub,lbA,NULL, nWSR  );

    real_t xOpt[1];
    CBF_.getPrimalSolution( xOpt );


    Pow_prev= xOpt[0] ;
    Time_tot +=dt ;

    return xOpt[0] ;

}

void cbf_tank::set_Pow_limt(realtype P_max){

    Pow_max=P_max  ;

}

realtype cbf_tank::smooth_sigma_map(realtype P, realtype P_min, realtype P_max,realtype k_f){



        if (P <= P_min) {
            alpha = 1;
        }
        else if (P >= P_max) {
            alpha = 0;
        }
        else {
            alpha = 0.5 + 0.5 * cos(PI * (P - P_min) / (P_max - P_min));
        }

          return k_f*  alpha ;
}


realtype cbf_tank::get_s_cbf(){
    return s_cbf ;
}

realtype cbf_tank::get_s_cbf_dot(){
    return s_cbf_dot ;
}


