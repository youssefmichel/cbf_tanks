
#ifndef BASEPLANNER_H_
#define BASEPLANNER_H_

#include <FastResearchInterface.h>
#include <FastResearchInterfaceTest.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include "dhdc.h"
#include "drdc.h"
#include "cbf_tank.h"
#include "utility_fri.h"
#include <TypeIRML.h>

#include "boost/filesystem.hpp"
using namespace boost::filesystem;


#ifndef LWR_JNT_NUM
#define LWR_JNT_NUM 7
#endif
#define NUMBER_OF_CART_DOFS	6
#define NUMBER_OF_JOINTS		7

using namespace std;
using namespace GeneralFunction ;

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000
#define SIZE_OF_TRANSFER_STRING					32
#define MIN_STIFFNESS 0.01

// Base class for interfacing the different tasks on the Kuka Robot


class BasePlanner{

protected:
    float   CartStiffnessValues_[FRI_CART_VEC],
    CommandedForcesAndTorques_ [NUMBER_OF_CART_DOFS] ,
    CartDampingValues_[FRI_CART_VEC];
    float currentCartPose_[FRI_CART_FRM_DIM] ;
    float     x_d_demonstration_[FRI_CART_FRM_DIM] ;
    float  CartPose_init_[FRI_CART_FRM_DIM] ;
    float currentJointPosition_[LWR_JNT_NUM];
    string packPath_ ;
    FastResearchInterface	*FRI_;
    realtype dt_ ;
    Vec x0_ ;
    Vec x_d_ ;
    Vec x_prev_ ;
    Vec x_dot_filt_prev_ ;
    MinJerk min_jerk_profile_;
    realtype K_transl_ ;
    realtype D_transl_ ;


public:
    BasePlanner(){


    }
    ~BasePlanner(){


    }

    bool Init(string packPath)  {

        packPath_=packPath ;
        FRI_ = new FastResearchInterface((packPath_ + "/data/Control-FRI-Driver_2ms.init").c_str());

        if(startCartImpedanceCtrl(FRI_,currentCartPose_)==0){
            FRI_->GetMeasuredCartPose(currentCartPose_);
            FRI_->GetMeasuredJointPositions(currentJointPosition_) ;
            FRI_->GetMeasuredCartPose( CartPose_init_);
            dt_=FRI_->GetFRICycleTime() ;


            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                if (i==0 || i==1 || i==2){
                    CartStiffnessValues_[i] =(float)0.01;
                    CartDampingValues_[i]=(float)0.0;
                }
                else{
                    CartStiffnessValues_[i] =(float)200.0 ;
                    CartDampingValues_[i]=(float)0.7;
                }
                CommandedForcesAndTorques_	[i]	=	(float)0.0;
            }
            FRI_->SetCommandedCartStiffness(  CartStiffnessValues_);
            FRI_->SetCommandedCartDamping(CartDampingValues_);
            FRI_->SetCommandedCartForcesAndTorques( CommandedForcesAndTorques_);

            x0_=GetTranslation(currentCartPose_) ; // 3*1 Vec
            x_d_=x0_ ;
            x_prev_=x0_ ;
            x_dot_filt_prev_=Vec::Zero(3) ;


            for(int i=0;i<12;i++){
                x_d_demonstration_[i]=CartPose_init_[i] ;
            }
            FRI_->SetCommandedCartPose(x_d_demonstration_);
            return true ;
        }
        else{
            ROS_ERROR("Failed to initalize Cartesian Impedance Controller !!") ;
            return false ;
        }
    }


        virtual void Run(realtype duration) =0;
        void Terminate()  {

        for(int i=0;i <3;i++){

            CommandedForcesAndTorques_[i]=0.0 ;

        }

        FRI_->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques_) ;
    }


    } ;

#endif
