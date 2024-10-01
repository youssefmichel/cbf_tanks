
#include "firstorderTank.h"


firstorder_tank::firstorder_tank(){

}

bool firstorder_tank::Initiliaze(double dt_) {
    if(! ros::param::get("Pow_max",Pow_max)) {
        ROS_WARN("Max Power Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_min",E_tank_min)) {
        ROS_WARN("Min Energy Param not found !!");
        return false;
    }
    if(! ros::param::get("EngTank_init",E_tank_init)) {
        ROS_WARN("Initial Energy Param not found !!");
        return false;
    }



    E_tank_max=E_tank_init ;
    E_tank= E_tank_init ;
    alpha=1 ;
    beta=1 ;

    dt=dt_;



    ROS_INFO("Tank Initialized properly");

    return true ;

}


realtype firstorder_tank::update_tank(realtype Pow_des) {

    if(Pow_des>0 & E_tank<E_tank_min){
        alpha =0 ;
    }
    else {
        alpha =1 ;
    }

    if(Pow_des<0 & E_tank>E_tank_max) {
       beta= 0 ;
    }
    else {
        beta =1 ;
    }

    if (Pow_des >0) {
        E_tank=E_tank - dt*alpha*Pow_des ;
    }
    else {
         E_tank=E_tank - dt*beta*Pow_des ;
    }

    return alpha ;

}


realtype firstorder_tank::get_tank_state(){
    return E_tank ;
}





bool firstorder_tank_cbf::Initiliaze(double dt_) {

    firstorder_tank::Initiliaze(dt_) ;
    set_xy_tank();
    cout<<"x_tank: "<<x_tank <<endl ;


    return true ;


}


realtype firstorder_tank_cbf::Get_Pow_opt(){
    return Pow_opt ;
}

realtype firstorder_tank_cbf::update_tank(realtype Pow_des) {


//     realtype u_t_des= -Pow_des /x_tank ;
//     Pow_opt=compute_u_cbf_1st(u_t_des) ;
//     x_tank = x_tank + dt* Pow_opt ;
//     if(Pow_des >0.001 ) {
//     alpha = abs(Pow_opt/u_t_des) ;
//            }

     Pow_opt=compute_u_cbf_1st(-Pow_des) ;
     E_tank=E_tank + dt*Pow_opt ;
     set_xy_tank();

     if(Pow_des >0.01 ) {
     alpha = abs(Pow_opt/Pow_des) ;
            }

    return alpha ;

}

realtype firstorder_tank_cbf::get_tank_state() {

return 0.5*x_tank*x_tank ;
}

void firstorder_tank_cbf::set_xy_tank() {

    x_tank = std::sqrt(2*E_tank) ;
    y_tank =x_tank ;
}

realtype firstorder_tank_cbf::compute_u_cbf_1st(realtype u_t_des) {


    int_t nWSR = 100;
    real_t lb[1] = { -100};
    real_t ub[1] = { 100};
    real_t H[1] = { 1.0  };
    real_t A[1] = {1}; //x_tank
    real_t f[1] = {  -u_t_des };
    realtype tmp_lba_1= -2.5* ( 0.5 * x_tank*x_tank - E_tank_min)   ;

    real_t lbA[1] = {  tmp_lba_1};
    real_t ubA[1] = { 500 };

    QProblem CBF_(1,1) ;
    CBF_.setPrintLevel(PL_NONE) ;
    CBF_.init( H,f,A,lb,ub,lbA,NULL, nWSR  );

    real_t xOpt[1];
    CBF_.getPrimalSolution( xOpt );


    return xOpt[0] ;
}

