#pragma once

#include "base_planner.h"

class KineticEnergyTaskPlanner:public BasePlanner{

private:

    int  ActivateTank_ ;
    realtype KE_max_;
    CBFTank CBF_Tank_ ;

public:
    bool Init(string packPath) {

        if (!BasePlanner::Init(packPath)) {
          return false ;
        }

        KE_max_=0.5 ;
        if(! ros::param::get("KE_max",KE_max_)) {
            ROS_WARN("Max Kin. Energy Param not found !!");
        }

        std::string tank_type="none" ;
        if(! ros::param::get("tank_type",tank_type)) {
            ROS_WARN("Tank Type Param not found !!");
        }

        ActivateTank_=1 ;
        if(tank_type=="cbf"){
            ActivateTank_=1 ;
        }
        else{
            ActivateTank_=0 ;
        }

        CBF_Tank_=CBFTank() ;
        CBF_Tank_.Initiliaze(0.002,ActivateTank_) ;
        K_transl_=800 ;
        if(! ros::param::get("K_transl",K_transl_)) {
                ROS_WARN("Stiffness Param not found !!");
            }
        x_d_(1)=x0_(1)-0.53 ;
        min_jerk_profile_=MinJerk(dt_,4, Vec::Zero(1) ,Vec::Ones(1) );

        ROS_INFO("Kinetic Energy task planner Initialized") ;
        return true ;
    }

    void Run(realtype duration) override{

        realtype T_max=duration ;
        realtype time_elap=0 ;
        int done = -1 ;
        realtype KE= 0 ;
        float **ptr_Inertia;
        ptr_Inertia = new float *[7];
        for(int i = 0; i <7; i++){
            ptr_Inertia[i] = new float[7];
        }

        FRI_->GetMeasuredJointPositions(currentJointPosition_);
        Vec q_prev=float_To_Vec(currentJointPosition_,7) ;
        Vec  q_dot_filt_prev= Vec::Zero(7) ;

        while(time_elap<T_max && (done==-1)) {

            if (dhdKbHit() && dhdKbGet()=='q') done = 1;

            FRI_->WaitForKRCTick();
            FRI_->GetMeasuredCartPose(currentCartPose_);
            Vec x=GetTranslation(currentCartPose_) ;
            FRI_->GetMeasuredJointPositions(currentJointPosition_);
            Vec q= float_To_Vec(currentJointPosition_,7) ;
            FRI_->GetCurrentMassMatrix(ptr_Inertia);
            Mat M=ConvertMassMatToMat(ptr_Inertia) ;
            Mat Rot_mat=GetRotationMatrix(currentCartPose_) ;

            Vec x_dot=(x-x_prev_)/dt_ ;
            x_prev_=x ;
            Vec q_dot=(q-q_prev)/dt_ ;
            q_prev=q ;

            Vec x_dot_filt=low_pass( x_dot, x_dot_filt_prev_,20,FRI_->GetFRICycleTime() ) ;
            x_dot_filt_prev_=x_dot_filt ;
            Vec q_dot_filt=low_pass( q_dot, q_dot_filt_prev,20,FRI_->GetFRICycleTime() ) ;
            q_dot_filt_prev=q_dot_filt ;

            min_jerk_profile_.Update(time_elap);
            Vec fac= K_transl_*min_jerk_profile_.GetDesiredPos() ;
            realtype k_des= fac(0);
            realtype D=2*0.7*sqrt(K_transl_) ;

            Vec F_des= k_des* (x_d_- x) ;
            realtype Pow_des= x_dot_filt.transpose()* F_des   ;
            realtype P_dissp= -x_dot_filt.transpose()* (D*x_dot_filt) ;
            realtype P_max=(1/dt_)*(KE_max_- (KE+  dt_*P_dissp ) ) ;
            CBF_Tank_.set_Pow_limt(-P_max);

            realtype alpha= CBF_Tank_.UpdateTank(Pow_des) ;
            Vec F_act=alpha*F_des   - D*x_dot_filt;
            realtype Pow_act= alpha*Pow_des ;
            Vec  Impedance_Force_tool=Rot_mat.transpose()*F_act ;

            for(int i=0;i <3;i++){
                CommandedForcesAndTorques_[i]=Impedance_Force_tool[i] ;
            }

            realtype P_tot_act=x_dot_filt.transpose()* F_act ;
            realtype KE_tot=0.5* q_dot_filt.transpose()*M*q_dot_filt;
            KE=KE + dt_*(P_tot_act) + (dt_*75*(KE_tot-KE)); // correct for drift
            x_d_demonstration_[3]=currentCartPose_[3]   ;
            x_d_demonstration_[7]=currentCartPose_[7]   ;
            x_d_demonstration_[11]=currentCartPose_[11] ;
            FRI_->SetCommandedCartPose(x_d_demonstration_);
            FRI_->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques_) ;
            time_elap+= dt_ ;

        }



   }

} ;



