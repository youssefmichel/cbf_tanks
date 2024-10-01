#ifndef FORCETASKPLANNER_H_
#define FORCETASKPLANNER_H_

#include "base_planner.h"

class ForceTaskPlanner:BasePlanner{

private:
    string Pow_rob_file_ ;
    string Tank_rob_file_ ;
    int  ActivateTank_ ;
    Vec F_des_ ;
    std::vector< std::vector <float> > Pow_rob_Vector_;
    std::vector< std::vector <float> > Tank_rob_Vector_;

public:
    bool Init(string packPath) {
        if( !BasePlanner::Init(packPath)){
            return false ;
        }
        printf("    Activate tank ? \n");
        char n ;
        n	=	WaitForKBCharacter(NULL);
        printf("\n\n\n");

        std::string ss; ss.push_back(n);
        Pow_rob_file_=packPath + "/data/force/Pow_rob_vid_ns"  +".txt" ;
        Tank_rob_file_=packPath + "/data/force/Tank_rob_vid_ns"  +".txt" ;
        ActivateTank_=1 ;
        if(n=='1'){
            // Activate tank
            Pow_rob_file_=packPath + "/data/force/Pow_rob_vid"+"ws2"  +".txt" ;
            Tank_rob_file_=packPath + "/data/force/Tank_rob_vid"+ "_ws2"  +".txt" ;
            ActivateTank_=1 ;
        }
        else{
            Pow_rob_file_=packPath + "/data/force/Pow_rob_vid"+"ns2"  +".txt" ;
            Tank_rob_file_=packPath + "/data/force/Tank_rob_vid"+ "_ns2"  +".txt" ;
            ActivateTank_=0 ;
        }
        realtype f_des=-10 ;
        if(! ros::param::get("f_des",f_des)) {
            ROS_WARN("Desired Force Param not found !!");

        }
        realtype D=100 ;
        if(! ros::param::get("D_transl",D)) {
            ROS_WARN("Desired Force Param not found !!");

        }
         F_des_= Vec::Zero(3) ; F_des_(2) = f_des;

    }

    void Run(realtype duration) override{
        realtype T_max=duration ;
        int it_=0 ;
        realtype t=0 ;
        int done = -1 ;
        x_d_(1)=x0_(1)+0.4 ;
        min_jerk_profile_ = new MinJerk(dt_,3, x0_ ,x_d_);
        CBFTank *MyCBF_Tank=new CBFTank() ; ;
        MyCBF_Tank->Initiliaze(dt_,ActivateTank_) ;


        while(t<T_max && (done==-1)) {

           if (dhdKbHit() && dhdKbGet()=='q') done = 1;

            FRI_->WaitForKRCTick();
            FRI_->GetMeasuredCartPose(currentCartPose_);
            Vec x=GetTranslation(currentCartPose_) ;
            Mat Rot_mat=GetRotationMatrix(currentCartPose_) ;
            Vec x_dot=(x-x_prev_)/dt_ ;
            x_prev_=x ;
            Vec x_dot_filt=low_pass( x_dot, x_dot_filt_prev_,20,FRI_->GetFRICycleTime() ) ;
            x_dot_filt_prev_=x_dot_filt ;

            min_jerk_profile_->Update(t);
            Vec x_d= min_jerk_profile_->GetDesiredPos() ;

            realtype Pow_des_force=x_dot_filt.transpose()*(F_des_) ;
            realtype alpha_force= MyCBF_Tank->UpdateTank(Pow_des_force) ;
            realtype Pow_act_force= alpha_force*Pow_des_force ;
            realtype alpha_traj=1 ;

            Vec x_d_act=x_d ;  x_d_act(1)=alpha_traj*x_d(1) ;
            Mat SelecMat=Mat::Identity(3,3) ; SelecMat(2,2)=0 ;
            Vec F_act=SelecMat*(800*(x_d-x) - 40*x_dot_filt) + alpha_force *F_des_ ;
            Vec  Impedance_Force_tool=Rot_mat.transpose()*F_act ;

            for(int i=0;i <3;i++){
                CommandedForcesAndTorques_[i]=Impedance_Force_tool[i] ;
            }

            x_d_demonstration_[3]=currentCartPose_[3]   ;
            x_d_demonstration_[7]=currentCartPose_[7]   ;
            x_d_demonstration_[11]=currentCartPose_[11] ;
            FRI_->SetCommandedCartPose(x_d_demonstration_);
            FRI_->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques_) ;


            Pow_rob_Vector_.push_back(std::vector <float>());
            Pow_rob_Vector_[it_].push_back(Pow_des_force);
            Pow_rob_Vector_[it_].push_back(Pow_act_force);
            Pow_rob_Vector_[it_].push_back(x_d(1));
            Pow_rob_Vector_[it_].push_back(x_d_act(1));
            Pow_rob_Vector_[it_].push_back(x(0));
            Pow_rob_Vector_[it_].push_back(x(1));
            Pow_rob_Vector_[it_].push_back(x(2));

            Tank_rob_Vector_.push_back(std::vector <float>());
            Tank_rob_Vector_[it_].push_back(MyCBF_Tank->get_s_cbf());
            Tank_rob_Vector_[it_].push_back(MyCBF_Tank->get_s_cbf_dot());
            Tank_rob_Vector_[it_].push_back(alpha_force);

            t+= dt_ ;
            it_ ++ ;

        }


    }

    void LogData() override{

        BasePlanner::LogData() ;
        saveVectorMatrixToFile(Pow_rob_file_,Pow_rob_Vector_);
        saveVectorMatrixToFile(Tank_rob_file_,Tank_rob_Vector_);

    }


} ;


#endif
