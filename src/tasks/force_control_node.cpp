
#include "base_planner.h"
#include "force_task_planner.h"



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
    int						ResultValue					=	0;

    FRI = new FastResearchInterface((packPath + "/data/Control-FRI-Driver_2ms.init").c_str());

    fprintf(stdout, "Check .\n");

    fprintf(stdout, "OK-OK \n");
    fflush(stdout);
    std::string initJointFile    = packPath + +"/data/" "InitAnglePos_force.txt";


    float   CartStiffnessValues[FRI_CART_VEC],
            CommandedForcesAndTorques [NUMBER_OF_CART_DOFS] ,
            CartDampingValues[FRI_CART_VEC];
    float currentCartPose[FRI_CART_FRM_DIM] ;
    float     x_d_demonstration[FRI_CART_FRM_DIM] ;
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

            printf("Gravity compensation... (please wait)\n");
            startGravityCompensation(FRI,50.0) ;
            break ;

        }
        case 'o':
        case 'O': {
            // move to initial desired orientation
            std::vector <double> v ;
            ROS_INFO("Going to a Desired Full Pose with a Min. Jerk Trajectory") ;
            if (! ros::param::get("q_init",v)){
                ROS_WARN("q_init Param Not Found !!") ;
            }

            FRI->GetMeasuredCartPose(currentCartPose);
            Vec x_d=GetTranslation(currentCartPose) ;
            x_d(2)=x_d(2)+0.1  ;
            Eigen::Matrix3d R_des=GetRotationMatrix(currentCartPose);
            Eigen::Quaterniond q_des(R_des);
            MoveCartesian_MinJerk_FullPose( FRI, 2, FRI->GetFRICycleTime(), x_d,  q_des) ;

            break ;}


        case 's':
        case 'S':{
            ForceTaskPlanner MyForcePlanner ;
            MyForcePlanner.Init(packPath) ;
            MyForcePlanner.Run(10.0);
            MyForcePlanner.LogData();

            break ;
        }


        case 'w':
        case 'W': {

            // Real Robot Implementation
            // Force Control Use Case
            if(startCartImpedanceCtrl(FRI,currentCartPose)==0){

                FRI->GetMeasuredCartPose(currentCartPose);
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
                realtype D_transl=40 ;
                if(! ros::param::get("D_transl",D_transl)) {
                    ROS_WARN("Damping Param not found !!");
                }
                realtype K_transl=800 ;
                if(! ros::param::get("K_transl",K_transl)) {
                    ROS_WARN("Stiffness Param not found !!");
                }

                Vec F_des= Vec::Zero(3) ; F_des(2) = f_des;
                realtype  dt=FRI->GetFRICycleTime() ;

                CBFTank MyCBF_Tank ;
                MyCBF_Tank.Initiliaze(dt,ActivateTank) ;
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
                Vec x_d=x0 ;  x_d(1)=x0(1)+0.4 ;
                Vec x_prev=x0 ;
                Vec x_dot_filt_prev=Vec::Zero(3) ;
                realtype T_max=10 ;
                realtype time_elap=0 ;
                int done = -1 ;
                MinJerk min_jerk_profile=MinJerk(dt,3, x0 ,x_d);


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
                    Vec x_d= min_jerk_profile.GetDesiredPos() ;
                    realtype Pow_des_force=x_dot_filt.transpose()*(F_des) ;
                    realtype alpha_force= MyCBF_Tank.UpdateTank(Pow_des_force) ;
                    realtype Pow_act_force= alpha_force*Pow_des_force ;
                    realtype alpha_traj=1 ;

                    Vec x_d_act=x_d ;  x_d_act(1)=alpha_traj*x_d(1) ;
                    Mat SelecMat=Mat::Identity(3,3) ; SelecMat(2,2)=0 ;
                    Vec F_act=SelecMat*(K_transl*(x_d-x) - D_transl*x_dot_filt) + alpha_force *F_des ;
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







