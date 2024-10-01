#include <ros/ros.h>
#include <ros/package.h>

#include <FastResearchInterface.h>
#include <FastResearchInterfaceTest.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <fstream>
#include <vector>
#include <OSAbstraction.h>
#include <sys/stat.h>
#include <cstring>
#include <LWRCartImpedanceController.h>

#include "dhdc.h"
#include "drdc.h"
#include <signal.h>





#ifndef LWR_JNT_NUM
#define LWR_JNT_NUM 7
#endif


using namespace std;

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#ifndef NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK
#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000
#endif

#define SIZE_OF_TRANSFER_STRING					32
#define LINEAR_VISCOSITY   30.0
#define MIN_STIFFNESS 0.01

using  namespace  Eigen;

volatile sig_atomic_t Exit_flag = 0;

int startCartImpedanceCtrl(FastResearchInterface *fri, float *commCartPose){
    unsigned int controlScheme = FastResearchInterface::CART_IMPEDANCE_CONTROL;
    int resultValue;
    if(fri->GetCurrentControlScheme() != controlScheme || !fri->IsMachineOK()){
        // Stop

        fri->StopRobot();

        fri->GetMeasuredCartPose(commCartPose);
        fri->SetCommandedCartPose(commCartPose);

        // Restart
        resultValue	= fri->StartRobot(controlScheme);

        if (resultValue != EOK){
            std::cout << "An error occurred during starting up the robot..." << std::endl;
            return -1;
        }
    }
    return 0;
}


int HD_gravityCompensation()
{


    if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
        //   printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        return -1 ;
    }
    return 1 ;

}


int startJointImpedanceCtrl(FastResearchInterface *fri, float *commJointPosition){
    unsigned int controlScheme = FastResearchInterface::JOINT_IMPEDANCE_CONTROL;
    int resultValue;
    if(fri->GetCurrentControlScheme() != controlScheme || !fri->IsMachineOK()){
        // Stop
        //cout << fri->WaitForKRCTick()<< endl;
        fri->StopRobot();


        fri->GetMeasuredJointPositions(commJointPosition);
        fri->SetCommandedJointPositions(commJointPosition);

        // Restart
        resultValue	= fri->StartRobot(controlScheme);
        if (resultValue != EOK){
            std::cout << "An error occurred during starting up the robot..." << std::endl;
            return -1;
        }
    }
    return 0 ;
}


int loadVectorMatrixFromFile (std::string fileName, int cols, vector<vector<float>> &outMat)
{
    ifstream in(fileName.data());
    if (!in)
    {
        cout << "No file found: " << fileName << endl;
        return -1;
    }
    int counter = 0;
    while (!in.eof())
    {
        outMat.push_back( vector <float>() );
        for (int j = 0; j < cols; ++j)
        {
            double readf;
            in >> readf;
            outMat[counter].push_back(readf);
        }
        counter++;
    }
    outMat.pop_back();
    in.close();
    return 0;
}

Vec SaturationFunc(Vec inp,float max){

    int size=inp.rows() ;
    Vec out(size) ;

    for(int i=0;i<size;i++){

        if(inp[i]>max){
            out[i]=max ;

        }
        else if(inp[i]< -max){
            out[i]=-max ;

        }
        else{
            out[i]=inp[i] ;

        }

    }
    return out ;

}



void saveVectorMatrixToFile (string fileName, vector < vector <float> > outMat)
{
    ofstream out(fileName.data());
    if (!out)
    {
        cout << "No file found: " << fileName << endl;
        return;
    }
    int rows = (int)outMat.size();
    int cols = (int)outMat[0].size();
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            out << outMat[i][j] << "\t";
        }
        out << endl;
    }
    out.close();
    return;
}
int initiliaze_HD(){


    if (dhdOpen () < 0) {
        printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep (2.0);
        return -1;
    }


    return 1 ;

}


float getSquaredDistance(float a[3], float b[3]){
    return (a[0]-b[0])*(a[0]-b[0]) +
            (a[1]-b[1])*(a[1]-b[1]) +
            (a[2]-b[2])*(a[2]-b[2]) ;
}

float low_pass(float signal, float prev_filt, float cutt_off, float cycle_time){

    return   ( ( signal*cutt_off*cycle_time)+ prev_filt )/(1+ cutt_off*cycle_time) ;

}

void Exit_Func(int sig){ // can be called asynchronously
    Exit_flag=1 ;
}


float compute_damping_factor(float F_ext,float F_ext_min,float F_ext_max){
    float f_s_min=0.0 ;
    if(F_ext>=F_ext_max)
        return 1;
    else if (F_ext<=F_ext_min)
        return f_s_min ;
    else
        return 1-( 0.5*(1+cos(PI/(F_ext_max - F_ext_min)*(F_ext - F_ext_min)))*(1-f_s_min) );
}



Mat GetRotationMatrix(float *CartPose){

    int tot_count=0 ;
    Mat Rot_Mat(3,3) ;
    for (int i=0;i<3;i++){

        int j=0 ;

        while(j<3){

            if ( !(tot_count==3 || tot_count==7|| tot_count==11 ) ){

                Rot_Mat(i,j)=CartPose[tot_count] ;
                j=j+1 ;

            }
            tot_count++ ;

        }



    }

    return Rot_Mat ;


}

Vec GetTranslation(float *CartPose){

    int tot_count=0 ;
    Vec transl(3) ;
    int i=0 ;

    while(i<3){

        if  (tot_count==3 || tot_count==7|| tot_count==11 ) {

            transl(i)=CartPose[tot_count] ;
            i++ ;


        }
        tot_count++ ;

    }

    return transl ;
}

Mat Convert_Jacobian_2Mat(float ** Jacobian){

    Mat output(6,7) ;

    for (int i = 0; i < 6; i++)
   {
      for (int j = 0; j < 7; j++)
       {
           //JacobianMatrix[i][j]  =   this->ReadData.data.FRIJacobianMatrix[(i==3)?(5):((i==5)?(3):(i))*NUMBER_OF_JOINTS+j];
           output(i,j)    =   Jacobian[i][j];
       }
   }

return output ;

}

Mat Convert_MassMat_2Mat(float ** Mass){

    Mat output(7,7) ;

    for (int i = 0; i < 7; i++)
   {
      for (int j = 0; j < 7; j++)
       {
           //JacobianMatrix[i][j]  =   this->ReadData.data.FRIJacobianMatrix[(i==3)?(5):((i==5)?(3):(i))*NUMBER_OF_JOINTS+j];
           output(i,j)    =   Mass[i][j];
       }
   }

return output ;

}



Vec float_2_Vec(float *inp,int size){

    Vec out(size) ;

    for (int i=0;i<size;i++){

out(i)=inp[i] ;
    }
    return out ;

}

Mat Tool_2_World_Jacobian(Mat Jac_tool,Mat Rot){
   Mat Jac_world=Jac_tool ;
   Mat zeros=MatrixXd::Zero(3,3);
   Mat Trans(6,6) ;
   Trans<<Rot,zeros ,
          zeros,Rot ;
   Jac_world=Trans *Jac_tool ;

 return Jac_world ;



}

