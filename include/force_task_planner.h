#ifndef FORCETASKPLANNER_H_
#define FORCETASKPLANNER_H_

#include "base_planner.h"

class ForceTaskPlanner: public BasePlanner{

private:

    int  ActivateTank_ ;
    Vec F_des_ ;
    CBFTank CBF_Tank_ ;



public:
    bool Init(string packPath) {

        if (!BasePlanner::Init(packPath)) {
          return false ;
        }
        std::string tank_type="none" ;
        if(! ros::param::get("tank_type",tank_type)) {
            ROS_WARN("Tank Type Param not found !!");
        }

        int ActivateTank=1 ;
        if(tank_type=="cbf"){
            ActivateTank=1 ;
        }
        else{
            ActivateTank=0 ;
        }

        realtype f_des=-10 ;
        if(! ros::param::get("f_des",f_des)) {
            ROS_WARN("Desired Force Param not found !!");
        }
        D_transl_=40 ;
        if(! ros::param::get("D_transl",D_transl_)) {
            ROS_WARN("Damping Param not found !!");
        }
        K_transl_=800 ;
        if(! ros::param::get("K_transl",K_transl_)) {
            ROS_WARN("Stiffness Param not found !!");
        }

         F_des_= Vec::Zero(3) ; F_des_(2) = f_des;
         dt_=FRI_->GetFRICycleTime() ;
         CBF_Tank_=CBFTank() ;
         CBF_Tank_.Initiliaze(dt_,ActivateTank_) ;
         x_d_(1)=x0_(1)+0.4 ;
         min_jerk_profile_ = MinJerk(dt_,3, x0_ ,x_d_);

         ROS_INFO("Force task planner Initialized") ;
         return true ;


    }

    void Run(realtype duration) override{

        realtype T_max=duration ;
        realtype time_elap=0 ;
        int done = -1 ;

        while(time_elap<T_max && (done==-1)) {

           if (dhdKbHit() && dhdKbGet()=='q') done = 1;

            FRI_->WaitForKRCTick();
            FRI_->GetMeasuredCartPose(currentCartPose_);
            Vec x=GetTranslation(currentCartPose_) ;
            Mat Rot_mat=GetRotationMatrix(currentCartPose_) ;
            Vec x_dot=(x-x_prev_)/dt_ ;
            x_prev_=x ;
            Vec x_dot_filt=low_pass( x_dot, x_dot_filt_prev_,20,FRI_->GetFRICycleTime() ) ;
            x_dot_filt_prev_=x_dot_filt ;

            min_jerk_profile_.Update(time_elap);
            Vec x_d= min_jerk_profile_.GetDesiredPos() ;
            realtype Pow_des_force=x_dot_filt.transpose()*(F_des_) ;
            realtype alpha_force= CBF_Tank_.UpdateTank(Pow_des_force) ;
            realtype Pow_act_force= alpha_force*Pow_des_force ;
            realtype alpha_traj=1 ;

            Vec x_d_act=x_d ;  x_d_act(1)=alpha_traj*x_d(1) ;
            Mat SelecMat=Mat::Identity(3,3) ; SelecMat(2,2)=0 ;
            Vec F_act=SelecMat*(K_transl_*(x_d-x) - D_transl_*x_dot_filt) + alpha_force *F_des_ ;
            Vec  Impedance_Force_tool=Rot_mat.transpose()*F_act ;

            for(int i=0;i <3;i++){
                CommandedForcesAndTorques_[i]=Impedance_Force_tool[i] ;
            }

            x_d_demonstration_[3]=currentCartPose_[3]   ;
            x_d_demonstration_[7]=currentCartPose_[7]   ;
            x_d_demonstration_[11]=currentCartPose_[11] ;
            FRI_->SetCommandedCartPose(x_d_demonstration_);
            FRI_->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques_) ;
            time_elap+= dt_ ;

        }

    }



} ;


#endif
