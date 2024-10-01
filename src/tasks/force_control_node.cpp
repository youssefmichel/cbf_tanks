


#include <FastResearchInterface.h>
#include <FastResearchInterfaceTest.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>


#include <vector>


#include "dhdc.h"
#include "drdc.h"
#include "CbfTank.h"



#include "UtilityFunctions.h"
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

    ros::init(argc, argv, "force_control_node");
    ros::NodeHandle nh;
    ROS_INFO("---------------  Force Control Task ----------------!!!") ;
    


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

    std::string initJointFile    = packPath + +"/data/" "InitAnglePos_force.txt";



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
       case 'i':
       case 'I':{

           printf("Priniting Robot poste Information.. (please wait)\n");
           FRI->GetMeasuredJointPositions(currentJointPosition);
           FRI->SetCommandedJointPositions(currentJointPosition);
           FRI->GetMeasuredCartPose(currentCartPose);
           FRI->SetCommandedCartPose(currentCartPose);
           cout<<"Joint Angles: \n "  << float_2_Vec(currentJointPosition,7) *(180/PI)	<<endl ;
           cout<<"Cartesian Position: \n "  << GetTranslation(currentCartPose)<<endl ;

           break ;
       }
       case 'g':
       case 'G':{

           printf("Gravity compensation... (please wait)\n");
           printf("       please enter Demo Number \n");
           char n ;
           n	=	WaitForKBCharacter(NULL);
           printf("\n\n\n");
           std::string ss; ss.push_back(n);

           

           std::vector< std::vector <float> > x_demo_Vector;
           std::vector< std::vector <float> > F_ext_Vector;
           std::vector< std::vector <float> > tau_ext_Vector;
           string x_demo_file=packPath + "/data/demos/X_demo_" +ss +".txt" ;
           string F_ext_demo_file=packPath + "/data/demos/F_ext_demo_" +ss +".txt" ;
           string tau_ext_demo_file=packPath + "/data/demos/tau_ext_demo_" +ss +".txt" ;

           if(startJointImpedanceCtrl(FRI, currentJointPosition)==0){

               printf("Gravity compensation Started \n");
               // Set stiffness
               for(i = 0; i < LWR_JNT_NUM; i++){
                   JointStiffnessValues[i] = (float)MIN_STIFFNESS; // max stiffness 0-2 -> 2000.0, max 3-5 200.0
               }

               FRI->SetCommandedJointStiffness(JointStiffnessValues);
               int it_= 0 ;

               int LoopValue = int(90.0 / FRI->GetFRICycleTime());
               int done_g=1 ;

               while (FRI->IsMachineOK()  && (it_<LoopValue) && (done_g == 1) ) {

                   FRI->WaitForKRCTick();

                   if (dhdKbHit() && dhdKbGet()=='q') done_g = -1;


                   FRI->GetMeasuredJointPositions(currentJointPosition);
                   FRI->SetCommandedJointPositions(currentJointPosition);
                   FRI->GetMeasuredCartPose(currentCartPose);
                   FRI->SetCommandedCartPose(currentCartPose);

                   Vec x_curr=GetTranslation(currentCartPose) ;
                   Matrix3d R_curr=GetRotationMatrix(currentCartPose) ;
                   Quaterniond q_curr(R_curr) ;

                   x_demo_Vector.push_back(std::vector <float>());
                   tau_ext_Vector.push_back(std::vector <float>());
                   F_ext_Vector.push_back(std::vector <float>());

                   x_demo_Vector[it_].push_back(x_curr(0));
                   x_demo_Vector[it_].push_back(x_curr(1));
                   x_demo_Vector[it_].push_back(x_curr(2));

                   x_demo_Vector[it_].push_back(q_curr.w());
                   x_demo_Vector[it_].push_back(q_curr.x());
                   x_demo_Vector[it_].push_back(q_curr.y());
                   x_demo_Vector[it_].push_back(q_curr.z());

                   FRI->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesTorques);
                   FRI->GetEstimatedExternalJointTorques(EstimatedExternalJointTorques);
                   for (int i=0;i <LWR_JNT_NUM ; i++ ){
                       tau_ext_Vector[it_].push_back(EstimatedExternalJointTorques[i]);
                   }
                   for (int i=0;i <FRI_CART_VEC ; i++ ){
                       F_ext_Vector[it_].push_back(EstimatedExternalCartForcesTorques[i]);
                   }


                   it_++;

               }
                saveVectorMatrixToFile(x_demo_file,x_demo_Vector);
                saveVectorMatrixToFile(F_ext_demo_file,F_ext_Vector);
                saveVectorMatrixToFile(tau_ext_demo_file,tau_ext_Vector);

               ROS_INFO("Saved file") ;

               cout<<"Joint Angles: \n "  << float_2_Vec(currentJointPosition,7) *(180/PI)	<<endl ;
               cout<<"Cartesian Position: \n "  << GetTranslation(currentCartPose)<<endl ;


           }
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

   //             x_d(0)= -0.402888 ;
   //             x_d(1)=0.369837;
                x_d(2)=x_d(2)+0.1  ;
                Eigen::Matrix3d R_des=GetRotationMatrix(currentCartPose);

                Eigen::Quaterniond q_des(R_des);
            //   Matrix3d R_init_VSDS= q_init_VSDS.normalized().toRotationMatrix() ;



                MoveCartesian_MinJerk_FullPose( FRI, 2, FRI->GetFRICycleTime(), x_d,  q_des) ;

          break ;}




       case 'w':
       case 'W': {

           // Real Robot Implementation
           // Time varying Trajectory Use Case

           FRI->GetMeasuredCartPose(currentCartPose);

           printf("       Activate tank ? \n");
           char n ;
           n	=	WaitForKBCharacter(NULL);
           printf("\n\n\n");
           std::string ss; ss.push_back(n);


          if(startCartImpedanceCtrl(FRI,currentCartPose)==0){

              realtype f_des=-10 ;
              if(! ros::param::get("f_des",f_des)) {
                  ROS_WARN("Desired Force Param not found !!");

              }
              realtype D=100 ;
              if(! ros::param::get("D_transl",D)) {
                  ROS_WARN("Desired Force Param not found !!");

              }
           Vec F_des= Vec::Zero(3) ; F_des(2) = f_des;

           std::vector< std::vector <float> > Pow_rob_Vector;
           std::vector< std::vector <float> > Tank_rob_Vector;


           string Pow_rob_file=packPath + "/data/force/Pow_rob_vid_ns"  +".txt" ;
           string Tank_rob_file=packPath + "/data/force/Tank_rob_vid_ns"  +".txt" ;
           int ActivateTank=1 ;

           if(n=='1'){
               // Activate tank
                Pow_rob_file=packPath + "/data/force/Pow_rob_vid"+"ws2"  +".txt" ;
                Tank_rob_file=packPath + "/data/force/Tank_rob_vid"+ "_ws2"  +".txt" ;
                ActivateTank=1 ;
           }
           else{
              Pow_rob_file=packPath + "/data/force/Pow_rob_vid"+"ns2"  +".txt" ;
               Tank_rob_file=packPath + "/data/force/Tank_rob_vid"+ "_ns2"  +".txt" ;
               ActivateTank=0 ;
           }



           realtype  dt=FRI->GetFRICycleTime() ;
           cout <<"Cycle time:"<<dt<<endl ;
           cbf_tank *MyCBF_Tank=new cbf_tank() ; ;
           MyCBF_Tank->Initiliaze(dt,ActivateTank) ;

           FRI->GetMeasuredJointPositions(currentJointPosition);
           FRI->GetMeasuredCartPose(currentCartPose);
           FRI->GetMeasuredCartPose( CartPose_init);
           for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
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
           Vec x_d=x0 ;  x_d(1)=x0(1)+0.4 ;
           Vec x_prev=x0 ;
           Vec x_dot_filt_prev=Vec::Zero(3) ;

           realtype T_max=10 ;

           int it_=0 ;
           realtype t=0 ;
           int done = -1 ;

           MinJerk *min_jerk_profile;
           min_jerk_profile = new MinJerk(dt,3, x0 ,x_d);
           Vec x_d_prev=x0 ;
           realtype alpha_prev=1 ;



           while(t<T_max && (done==-1)) {

               if (dhdKbHit() && dhdKbGet()=='q') done = 1;

               FRI->WaitForKRCTick();
               FRI->GetMeasuredCartPose(currentCartPose);
               Vec x=GetTranslation(currentCartPose) ;
               Mat Rot_mat=GetRotationMatrix(currentCartPose) ;
               Vec x_dot=(x-x_prev)/dt ;
               x_prev=x ;
               Vec x_dot_filt=low_pass( x_dot, x_dot_filt_prev,20,FRI->GetFRICycleTime() ) ;
               x_dot_filt_prev=x_dot_filt ;

               min_jerk_profile->Update(t);
               Vec x_d= min_jerk_profile->GetDesiredPos() ;
               Vec x_d_dot=(x_d-x_d_prev)/dt ; x_d_prev=x_d ;



               realtype Pow_des_traj=x_dot_filt.transpose()*(D*x_d_dot) ;
               realtype Pow_des_force=x_dot_filt.transpose()*(F_des) ;

               realtype alpha_force= MyCBF_Tank->update_tank(Pow_des_force) ;
             //  alpha_force=low_pass(alpha_force,alpha_prev,20,dt) ;
              // alpha_prev=alpha_force ;

               realtype Pow_act_force= alpha_force*Pow_des_force ;
               realtype alpha_traj=1 ;

              Vec x_d_act=x_d ;  x_d_act(1)=alpha_traj*x_d(1) ;


              Mat SelecMat=Mat::Identity(3,3) ; SelecMat(2,2)=0 ;
           // Vec F_act=SelecMat*D*(alpha_traj*x_d_dot-x_dot_filt) + alpha_force *F_des  ;

              Vec F_act=SelecMat*(800*(x_d-x) - 40*x_dot_filt) + alpha_force *F_des ;


              Vec  Impedance_Force_tool=Rot_mat.transpose()*F_act ;

              for(int i=0;i <3;i++){
                  CommandedForcesAndTorques[i]=Impedance_Force_tool[i] ;
              }

              x_d_demonstration[3]=currentCartPose[3]   ;
              x_d_demonstration[7]=currentCartPose[7]   ;
              x_d_demonstration[11]=currentCartPose[11] ;
              FRI->SetCommandedCartPose(x_d_demonstration);
              FRI->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques) ;



               Pow_rob_Vector.push_back(std::vector <float>());
               Pow_rob_Vector[it_].push_back(Pow_des_force);
               Pow_rob_Vector[it_].push_back(Pow_act_force);
               Pow_rob_Vector[it_].push_back(x_d(1));
               Pow_rob_Vector[it_].push_back(x_d_act(1));
               Pow_rob_Vector[it_].push_back(x_d_dot(1));
               Pow_rob_Vector[it_].push_back(x(0));
               Pow_rob_Vector[it_].push_back(x(1));
               Pow_rob_Vector[it_].push_back(x(2));

               Tank_rob_Vector.push_back(std::vector <float>());
               Tank_rob_Vector[it_].push_back(MyCBF_Tank->get_s_cbf());
               Tank_rob_Vector[it_].push_back(MyCBF_Tank->get_s_cbf_dot());
               Tank_rob_Vector[it_].push_back(alpha_force);

               t+= dt ;
               it_ ++ ;

           }

           for(int i=0;i <3;i++){

               CommandedForcesAndTorques[i]=0.0 ;

           }

         FRI->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques) ;

         saveVectorMatrixToFile(Pow_rob_file,Pow_rob_Vector);
         saveVectorMatrixToFile(Tank_rob_file,Tank_rob_Vector);

           }


          break ;}



       case 'q':
       case 'Q':



           ResultValue=FRI->StopRobot() ;

           delete FRI;
           sleep(3);
           return(EXIT_SUCCESS);
       }
       }




    }







