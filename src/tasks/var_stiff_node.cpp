


#include <FastResearchInterface.h>
#include <FastResearchInterfaceTest.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <chrono>

#include <vector>


#include "dhdc.h"
#include "drdc.h"
#include "cbf_tank.h"
#include "first_order_tank.h"

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
using namespace StiffnessProfiles;
using namespace GeneralFunction ;

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000
#define SIZE_OF_TRANSFER_STRING					32
#define MIN_STIFFNESS 0.01


//*******************************************************************************************
// main()

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "cbftank_main");
    ros::NodeHandle nh;
    ROS_INFO("---------------  Motion Control Task ----------------!!!") ;
    string tank_type ;
    nh.getParam("/tank_type",tank_type) ;
    FastResearchInterface	*FRI;
    std::string packPath = ros::package::getPath("cbf_tanks_pkg");
    std::cout << packPath << "\n";

    bool					Run							=	true;
    int						ResultValue					=	0;

    FRI = new FastResearchInterface((packPath + "/data/Control-FRI-Driver_2ms.init").c_str());

    fprintf(stdout, "Check .\n");
    fprintf(stdout, "OK-OK \n");
    fflush(stdout);
    std::string initJointFile    = packPath + +"/data/" "InitAnglePos.txt";

   float    CartStiffnessValues[FRI_CART_VEC],
            CommandedForcesAndTorques [NUMBER_OF_CART_DOFS] ,
            CartDampingValues[FRI_CART_VEC];

    float currentCartPose[FRI_CART_FRM_DIM] ;
    float     x_d_demonstration[FRI_CART_FRM_DIM] ;
    FRI->SetCommandedCartPose(x_d_demonstration);
    float  CartPose_init[FRI_CART_FRM_DIM] ;
    float currentJointPosition[LWR_JNT_NUM];


    while (Run)
    {
        std::string DataPathEnding;
        cout<<"\n"<<"Which Test you want to execute?\n"<<"\n";
        cout<<"Please Choose a Character\n";
        cin>>DataPathEnding;
        cout<<"\n\n";
        char c	=	WaitForKBCharacter(NULL);
        printf("\n\n\n");
        printf("Starting.....");

        switch (c)
        {
        case 'h':
        case 'H':{

            printf("Going to Home position... (please wait)\n");
            RunJointTrajectory(FRI, initJointFile);
            break ;
        }

        case 'g':
        case 'G':{

           startGravityCompensation(FRI,30.0 ) ;
                   break ;

            }


        case 'o':
        case 'O': {
            // move to initial desired orientation
            std::vector <double> v ;
            Quaterniond q_init_VSDS ;
            ROS_INFO("Going to a Desired Full Pose with a Min. Jerk Trajectory") ;

            if (! ros::param::get("q_init",v)){
                ROS_WARN("q_init Param Not Found !!") ;
            }
            q_init_VSDS.w()=0.0018134 ;
            q_init_VSDS.x()=0.858707 ;
            q_init_VSDS.y()=0.512366 ;
            q_init_VSDS.z()=-0.00999159	 ;
            FRI->GetMeasuredCartPose(currentCartPose);
            Vec x_d=GetTranslation(currentCartPose) ;
            x_d(2)=x_d(2)+0.1 ;
            Matrix3d R_init= GetRotationMatrix(currentCartPose) ;
            Quaterniond q_init(R_init) ;
            MoveCartesian_MinJerk_FullPose( FRI, 2, FRI->GetFRICycleTime(), x_d,  q_init) ;
            break ;}

        case 's':
        case 'S': {

            // Time varying Stiffness Use Case
            FRI->GetMeasuredCartPose(currentCartPose);
            printf("       please enter Demo Number \n");
            char n ;
            n	=	WaitForKBCharacter(NULL);
            printf("\n\n\n");
            std::string ss; ss.push_back(n);
            if(startCartImpedanceCtrl(FRI,currentCartPose)==0){

                char order='2' ;
                int Activate_CBF_Tank=1 ;
                if (tank_type=="first") {
                    order= '1' ;
                }
                else if(tank_type== "firstcbf") {
                    order= '3' ;
                }
                else if(tank_type== "none")  {
                    Activate_CBF_Tank=0 ;
                }
                else {
                    order = 2;
                }


                realtype K_transl=800 ;
                if(! ros::param::get("K_transl",K_transl)) {
                    ROS_WARN("Stiffness Param not found !!");
                }

                realtype  dt=FRI->GetFRICycleTime() ;

                CBFTank *MyCBF_Tank=new CBFTank() ;
                MyCBF_Tank->Initiliaze(0.002,Activate_CBF_Tank) ;

                FirstOrderTank *MyFirstOrder_Tank=new FirstOrderTank() ;
                MyFirstOrder_Tank->Initiliaze(0.002,0) ;

                FirstOrderTankCBF *MyFirstOrder_Tank_cbf=new FirstOrderTankCBF() ;
                MyFirstOrder_Tank_cbf->Initiliaze(0.002,0) ;

                FRI->GetMeasuredJointPositions(currentJointPosition);
                FRI->GetMeasuredCartPose(currentCartPose);
                FRI->GetMeasuredCartPose( CartPose_init);

                for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
                {
                    if (i==0 || i==1 || i==2){
                        CartStiffnessValues[i] =(float)0.01;
                        CartDampingValues[i]=(float)0.0;
                    }
                    else{
                        CartStiffnessValues[i] =(float)200.0 ;
                        CartDampingValues[i]=(float)0.7;

                    }
                    CommandedForcesAndTorques	[i]	=	(float)0.0;
                }
                FRI->SetCommandedCartStiffness(  CartStiffnessValues);
                FRI->SetCommandedCartDamping(CartDampingValues);
                FRI->SetCommandedCartForcesAndTorques( CommandedForcesAndTorques);
                for(int i=0;i<12;i++){
                    x_d_demonstration[i]=CartPose_init[i] ;
                }
                FRI->SetCommandedCartPose(x_d_demonstration);

                Vec x0=GetTranslation(currentCartPose) ; // 3*1 Vec
                Vec x_d=x0 ;  x_d(1)=x0(1)-0.45 ;
                Vec x_prev=x0 ;
                Vec x_dot_filt_prev=Vec::Zero(3) ;

                realtype T_max=10 ;
                realtype time_elap=0 ;
                int done = -1 ;

                MinJerk min_jerk_profile=MinJerk(dt,4, Vec::Zero(1) ,Vec::Ones(1) );;
                realtype alpha=1 ;
                realtype Pow_des_filt_prev=0 ;

                while(time_elap<T_max && (done==-1)) {

                    if (dhdKbHit() && dhdKbGet()=='q') done = 1;

                    FRI->WaitForKRCTick();
                    FRI->GetMeasuredCartPose(currentCartPose);
                    Vec x=GetTranslation(currentCartPose) ;
                    Mat Rot_mat=GetRotationMatrix(currentCartPose) ;

                    Vec x_dot=(x-x_prev)/dt ;
                    x_prev=x ;
                    Vec x_dot_filt=low_pass( x_dot, x_dot_filt_prev,20,FRI->GetFRICycleTime() ) ;
                    x_dot_filt_prev=x_dot_filt ;

                    min_jerk_profile.Update(time_elap);
                    Vec fac= K_transl*min_jerk_profile.GetDesiredPos() ;
                    realtype k_des= fac(0);
                    Vec F_des= k_des* (x_d- x) ;
                    realtype Pow_des= x_dot_filt.transpose()* F_des   ;
                    Pow_des=low_pass( Pow_des, Pow_des_filt_prev,10,FRI->GetFRICycleTime() ) ;
                    Pow_des_filt_prev=Pow_des ;

                    switch(order) {
                    case '1': {
                        alpha= MyFirstOrder_Tank->UpdateTank(Pow_des) ;
                        break ;
                    }
                    case '2':{
                        alpha= MyCBF_Tank->UpdateTank(Pow_des) ;
                        break ;
                    }

                    case '3':{
                        alpha= MyFirstOrder_Tank_cbf->UpdateTank(Pow_des) ;
                        break ;
                    }

                    }

                    realtype D=2*0.7*sqrt(500) ;
                    Vec F_act=alpha*F_des   - D*x_dot_filt;
                    realtype Pow_act= alpha*Pow_des ;
                    Vec  Impedance_Force_tool=Rot_mat.transpose()*F_act ;

                    for(int i=0;i <3;i++){
                        CommandedForcesAndTorques[i]=Impedance_Force_tool[i] ;
                    }

                    x_d_demonstration[3]=currentCartPose[3]   ;
                    x_d_demonstration[7]=currentCartPose[7]   ;
                    x_d_demonstration[11]=currentCartPose[11] ;
                    FRI->SetCommandedCartPose(x_d_demonstration);
                    FRI->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques) ;
                    time_elap+= dt ;

                }
                for(int i=0;i <3;i++){
                    CommandedForcesAndTorques[i]=0.0 ;
                }

                FRI->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques) ;

            }

            break ;
        }

        case 'q':
        case 'Q':

            ResultValue=FRI->StopRobot() ;
            delete FRI;
            sleep(1);
            return(EXIT_SUCCESS);
        }
    }

}










