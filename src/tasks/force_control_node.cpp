
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
            MyForcePlanner.Terminate();

            break ;
        }


        case 'q':
        case 'Q':

            ResultValue=FRI->StopRobot() ;
            delete FRI;
            sleep(3);
            return(EXIT_SUCCESS);
        }
    }




}







