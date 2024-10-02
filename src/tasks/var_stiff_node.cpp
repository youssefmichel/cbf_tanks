
#include "varStiff_task_planner.h"

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
        case 'S':{
            VarStiffTaskPlanner MyVarStiffPlanner ;
            MyVarStiffPlanner.Init(packPath) ;
            MyVarStiffPlanner.Run(10.0);
            MyVarStiffPlanner.Terminate();

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










