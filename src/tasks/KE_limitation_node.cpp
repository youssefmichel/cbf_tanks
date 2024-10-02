


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


#include "kinetic_lim_task_planner.h"

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

    ros::init(argc, argv, "KE_limitation_node");
    ros::NodeHandle nh;
    ROS_INFO("---------------  KE Energy Limitations Task ----------------!!!") ;



    FastResearchInterface	*FRI;
    std::string packPath = ros::package::getPath("cbf_tanks_pkg");
    std::cout << packPath << "\n";

    bool					Run							=	true;

    char					c							=	0
            ,	d							=	0;

    unsigned int			ControlScheme				=	FastResearchInterface::JOINT_POSITION_CONTROL;
    int i = 0;

    int						ResultValue					=	0;

    FRI = new FastResearchInterface((packPath + "/data/Control-FRI-Driver_2ms.init").c_str());

    fprintf(stdout, "Check .\n");

    fprintf(stdout, "OK-OK \n");
    fflush(stdout);

    string Save_Data_dir=packPath + "/data/" ;

    std::string initJointFile    = packPath + +"/data/" "InitAnglePos.txt";



    if ( !exists( Save_Data_dir ) ) { // Check if src folder exists
        boost::filesystem::create_directory(Save_Data_dir);
    }



    float  JointStiffnessValues[LBR_MNJ],
            EstimatedExternalCartForcesTorques[FRI_CART_VEC],
            EstimatedExternalJointTorques[LBR_MNJ],
            CartStiffnessValues[FRI_CART_VEC],
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
        c	=	WaitForKBCharacter(NULL);
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

            printf("Gravity compensation... (please wait)\n");
            printf("       please enter Demo Number \n");
            startGravityCompensation(FRI, 50.0) ;
             break ;
        }


        case 'o':
        case 'O': {
            // move to initial desired orientation
            std::vector <double> v ;
            Quaterniond q_init_des ;

            ROS_INFO("Going to a Desired Full Pose with a Min. Jerk Trajectory") ;

            if (! ros::param::get("q_init",v)){
                ROS_WARN("q_init Param Not Found !!") ;
            }

            q_init_des.w()=0.0018134 ;
            q_init_des.x()=0.858707 ;
            q_init_des.y()=0.512366 ;
            q_init_des.z()=-0.00999159	 ;
            FRI->GetMeasuredCartPose(currentCartPose);
            Vec x_d=GetTranslation(currentCartPose) ;
            MoveCartesian_MinJerk_FullPose( FRI, 2, FRI->GetFRICycleTime(), x_d,  q_init_des) ;
            break ;}

        case 's':
        case 'S':{
            KineticEnergyTaskPlanner MyKineticPlanner ;
            MyKineticPlanner.Init(packPath) ;
            MyKineticPlanner.Run(10.0);
            MyKineticPlanner.Terminate();

            break ;
        }


        case 'w':
        case 'W': {

            // Kinetic Energy Limitation
            FRI->GetMeasuredCartPose(currentCartPose);
            if(startCartImpedanceCtrl(FRI,currentCartPose)==0){

                realtype KE_max=0.5 ;
                if(! ros::param::get("KE_max",KE_max)) {
                    ROS_WARN("Max Kin. Energy Param not found !!");
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

                realtype  dt=FRI->GetFRICycleTime() ;
                CBFTank *MyCBF_Tank=new CBFTank() ; ;
                MyCBF_Tank->Initiliaze(0.002,ActivateTank) ;

                FRI->GetMeasuredJointPositions(currentJointPosition);
                FRI->GetMeasuredCartPose(currentCartPose);
                FRI->GetMeasuredCartPose( CartPose_init);

                realtype K_transl=800 ;
                if(! ros::param::get("K_transl",K_transl)) {
                    ROS_WARN("Stiffness Param not found !!");
                }

                for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
                {
                    if (i<3){
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
                Vec x0=GetTranslation(currentCartPose) ;
                Vec x_d=x0 ;  x_d(1)=x0(1)-0.53 ;
                Vec x_prev=x0 ;
                Vec x_dot_filt_prev=Vec::Zero(3) ;

                realtype T_max=10 ;
                realtype time_elap=0 ;
                int done = -1 ;

                MinJerk min_jerk_profile=MinJerk(dt,4, Vec::Zero(1) ,Vec::Ones(1) );
                realtype KE= 0 ;
                float **ptr_Inertia;
                ptr_Inertia = new float *[7];
                for(int i = 0; i <7; i++){
                    ptr_Inertia[i] = new float[7];
                }

                FRI->GetMeasuredJointPositions(currentJointPosition);
                Vec q_prev=float_To_Vec(currentJointPosition,7) ;
                Vec  q_dot_filt_prev= Vec::Zero(7) ;

                while(time_elap<T_max && (done==-1)) {

                    if (dhdKbHit() && dhdKbGet()=='q') done = 1;

                    FRI->WaitForKRCTick();
                    FRI->GetMeasuredCartPose(currentCartPose);
                    Vec x=GetTranslation(currentCartPose) ;
                    FRI->GetMeasuredJointPositions(currentJointPosition);
                    Vec q= float_To_Vec(currentJointPosition,7) ;
                    FRI->GetCurrentMassMatrix(ptr_Inertia);
                    Mat M=ConvertMassMatToMat(ptr_Inertia) ;
                    Mat Rot_mat=GetRotationMatrix(currentCartPose) ;

                    Vec x_dot=(x-x_prev)/dt ;
                    x_prev=x ;
                    Vec q_dot=(q-q_prev)/dt ;
                    q_prev=q ;

                    Vec x_dot_filt=low_pass( x_dot, x_dot_filt_prev,20,FRI->GetFRICycleTime() ) ;
                    x_dot_filt_prev=x_dot_filt ;
                    Vec q_dot_filt=low_pass( q_dot, q_dot_filt_prev,20,FRI->GetFRICycleTime() ) ;
                    q_dot_filt_prev=q_dot_filt ;

                    min_jerk_profile.Update(time_elap);
                    Vec fac= K_transl*min_jerk_profile.GetDesiredPos() ;
                    realtype k_des= fac(0);
                    realtype D=2*0.7*sqrt(K_transl) ;

                    Vec F_des= k_des* (x_d- x) ;
                    realtype Pow_des= x_dot_filt.transpose()* F_des   ;
                    realtype P_dissp= -x_dot_filt.transpose()* (D*x_dot_filt) ;
                    realtype P_max=(1/dt)*(KE_max- (KE+  dt*P_dissp ) ) ;
                    MyCBF_Tank->set_Pow_limt(-P_max);

                    realtype alpha= MyCBF_Tank->UpdateTank(Pow_des) ;
                    Vec F_act=alpha*F_des   - D*x_dot_filt;
                    realtype Pow_act= alpha*Pow_des ;
                    Vec  Impedance_Force_tool=Rot_mat.transpose()*F_act ;

                    for(int i=0;i <3;i++){
                        CommandedForcesAndTorques[i]=Impedance_Force_tool[i] ;
                    }

                    realtype P_tot_act=x_dot_filt.transpose()* F_act ;
                    realtype KE_tot=0.5* q_dot_filt.transpose()*M*q_dot_filt;
                    KE=KE + dt*(P_tot_act) + (dt*75*(KE_tot-KE)); // correct for drift
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


            break ;}

        case 'q':
        case 'Q':

            ResultValue=FRI->StopRobot() ;
            delete FRI;
            sleep(1);
            return(EXIT_SUCCESS);
    }
}

}







