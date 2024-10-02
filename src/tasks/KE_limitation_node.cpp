




#include "kinetic_lim_task_planner.h"








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

        case 'q':
        case 'Q':

            ResultValue=FRI->StopRobot() ;
            delete FRI;
            sleep(1);
            return(EXIT_SUCCESS);
    }
}

}







