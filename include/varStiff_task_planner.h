#pragma once

#include "base_planner.h"
#include "first_order_tank.h"

class VarStiffTaslPlanner:public BasePlanner{

private:

    int  ActivateTank_ ;
    CBFTank CBF_Tank_ ;
    char order_ ;
    FirstOrderTank FirstOrder_Tank_ ;
    FirstOrderTankCBF FirstOrder_Tank_cbf_ ;

public:
    bool Init(string packPath) {
        if (!BasePlanner::Init(packPath)) {
          return false ;
        }

        order_='2' ;
        int Activate_CBF_Tank=1 ;
        string tank_type="cbf" ;
        if (tank_type=="first") {
            order_= '1' ;
        }
        else if(tank_type== "firstcbf") {
            order_= '3' ;
        }
        else if(tank_type== "none")  {
            Activate_CBF_Tank=0 ;
        }
        else {
            order_ = 2;
        }

        K_transl_=800 ;
        if(! ros::param::get("K_transl",K_transl_)) {
            ROS_WARN("Stiffness Param not found !!");
        }

          CBF_Tank_=CBFTank() ;
          CBF_Tank_.Initiliaze(dt_,Activate_CBF_Tank) ;


          FirstOrder_Tank_.Initiliaze(0.002,0) ;
          FirstOrder_Tank_cbf_.Initiliaze(0.002,0) ;
          x_d_(1)=x0_(1)-0.45 ;
          min_jerk_profile_=MinJerk(dt_,4, Vec::Zero(1) ,Vec::Ones(1) );;



        ROS_INFO("VarStiff Task Planner Initialized") ;
        return true ;
    }

        void Run(realtype duration) override{

            realtype T_max=duration ;
            realtype time_elap=0 ;
            int done = -1 ;
            realtype alpha=1 ;
            realtype Pow_des_filt_prev=0 ;

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
                Vec fac= K_transl_*min_jerk_profile_.GetDesiredPos() ;
                realtype k_des= fac(0);
                Vec F_des= k_des* (x_d_- x) ;
                realtype Pow_des= x_dot_filt.transpose()* F_des   ;
                Pow_des=low_pass( Pow_des, Pow_des_filt_prev,10,FRI_->GetFRICycleTime() ) ;
                Pow_des_filt_prev=Pow_des ;

                switch(order_) {
                case '1': {
                    alpha= FirstOrder_Tank_.UpdateTank(Pow_des) ;
                    break ;
                }
                case '2':{
                    alpha= CBF_Tank_.UpdateTank(Pow_des) ;
                    break ;
                }

                case '3':{
                    alpha= FirstOrder_Tank_cbf_.UpdateTank(Pow_des) ;
                    break ;
                }

                }

                realtype D=2*0.7*sqrt(500) ;
                Vec F_act=alpha*F_des   - D*x_dot_filt;
                realtype Pow_act= alpha*Pow_des ;
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

};
