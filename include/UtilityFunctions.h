#pragma once
#include <ros/ros.h>
#include <ros/package.h>

#include <FastResearchInterface.h>

#include <time.h>


#include <stdlib.h>

#include <math.h>
#include <string.h>
#include <errno.h>
#include <fstream>
#include <vector>
#include <OSAbstraction.h>


#include "Utility.h"

#include "min_jerk.h"
#include "dhdc.h"
#include "drdc.h"
#include <signal.h>

#ifndef LWR_JNT_NUM
#define LWR_JNT_NUM 7
#endif



//const realtype pi=         3.141592653589793238462643383279502884 /* pi */;

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



int startCartImpedanceCtrl(FastResearchInterface *fri, float *commCartPose) ;

int startJointImpedanceCtrl(FastResearchInterface *fri, float *commJointPosition) ;
int loadVectorMatrixFromFile (std::string fileName, int cols, vector<vector<float>> &outMat) ;
Vec SaturationFunc(Vec inp,float max) ;
void saveVectorMatrixToFile (string fileName, vector < vector <float> > outMat) ;


float low_pass(float signal, float prev_filt, float cutt_off, float cycle_time) ;

Mat GetRotationMatrix(float *CartPose) ;
Vec GetTranslation(float *CartPose) ; 
Mat Convert_Jacobian_2Mat(float ** Jacobian) ;
Mat Convert_MassMat_2Mat(float ** Mass) ;
Vec float_2_Vec(float *inp,int size) ;
Mat Tool_2_World_Jacobian(Mat Jac_tool,Mat Rot) ;
void Set_Desired_Pose_FRI(Mat Rot_d,Vec x_d,float *CartPose_d) ;
Vec Quart_Orient_err(Mat R_a, Mat R_d);
Vec low_pass(Vec signal, Vec prev_filt, float cutt_off, float cycle_time) ;
vector< std::vector <float> > Kuka_MoveCartesian_MinJerk(FastResearchInterface *FRI, float tot_time, float dt, Vec x_df) ;
int MoveCartesian_MinJerk_FullPose(FastResearchInterface *FRI, float tot_time, float dt, Vec x_df, Quaterniond q_df) ;
int TrackTrajectory_JointImpedance(FastResearchInterface *FRI,string des_traj_file, string Save_Data_dir ) ;
int RunJointTrajectory(FastResearchInterface *FRI, std::string InitJointPosFile) ;
int RunJointTrajectory(FastResearchInterface *FRI, float *TargetJointValuesInRad) ;
int RunCartesianTrajectory(FastResearchInterface *FRI, float *TargetJointValuesInRad) ;
//int MoveCartesian_Trajectory_FullPose(FastResearchInterface *FRI, string des_traj_file,string Save_Data_dir) ;
