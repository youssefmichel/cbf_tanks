//  ---------------------- Doxygen info ----------------------
//! \file RunTrajectorySimple.cpp
//!
//! \brief
//! Implementation file for executing motion trajectories in the test
//! program for the class FastResearchInterface
//!
//! \details
//! The class FastResearchInterface provides a basic low-level interface
//! to the KUKA Light-Weight Robot IV For details, please refer to the file
//! FastResearchInterface.h.
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see 
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date November 2011
//!
//! \version 1.0
//!
//!	\author Torsten Kroeger, tkr@stanford.edu
//!
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <FastResearchInterface.h>
#include <Console.h>
#include <errno.h>
#include <string.h>
#include <OSAbstraction.h>
#include <TypeIRML.h>

#include <fstream>
#include <iostream>

#include <FastResearchInterfaceTest.h>

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif

#define NUMBER_OF_JOINTS			7

int RunJointTrajectory(FastResearchInterface *FRI, std::string InitJointPosFile)
{
    unsigned int i = 0;

    int	ResultValue	= 0;

    float JointValuesInRad[NUMBER_OF_JOINTS];
    float TargetJointValuesInDeg[NUMBER_OF_JOINTS];

    memset(JointValuesInRad, 0x0, NUMBER_OF_JOINTS * sizeof(float));
    memset(TargetJointValuesInDeg, 0x0, NUMBER_OF_JOINTS * sizeof(float));

    double CycleTime = FRI->GetFRICycleTime();//0.002;

    std::cout << "CycleTime: " << CycleTime << std::endl;

    TypeIRML *RML =	NULL;

    TypeIRMLInputParameters	*IP	= NULL;

    TypeIRMLOutputParameters *OP = NULL;

    RML	= new TypeIRML(NUMBER_OF_JOINTS, CycleTime);

    IP = new TypeIRMLInputParameters(NUMBER_OF_JOINTS);

    OP = new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

    // Load Desired Joint Position
    std::ifstream DesPosFile;
    DesPosFile.open(InitJointPosFile.c_str());
    if(DesPosFile)
    {
        for(i = 0; i < NUMBER_OF_JOINTS; ++i)
        {
            DesPosFile >> TargetJointValuesInDeg[i];
        }
        DesPosFile.close();
    }
    else
    {
        std::cout << " File " << InitJointPosFile << " not found!!!" << std::endl;
        exit(1);
    }

    if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
    {
        printf("Program is going to stop the robot.\n");
        FRI->StopRobot();

        FRI->GetMeasuredJointPositions(JointValuesInRad);
        FRI->SetCommandedJointPositions(JointValuesInRad);

        printf("Restarting the joint position control scheme.\n");
        ResultValue	= FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);

        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            printf("An error occurred during starting up the robot...\n");
            delete	RML;
            delete	IP;
            delete	OP;

            return -1;
        }
    }

    std::cout << "Target joint angles: [";
    for(i = 0; i < NUMBER_OF_JOINTS; ++i){
        std::cout << TargetJointValuesInDeg[i] << ", ";
    }
    std::cout << "]"<<std::endl;

    FRI->GetMeasuredJointPositions(JointValuesInRad);

    std::cout << "Current joint angles: [";
    for(i = 0; i < NUMBER_OF_JOINTS; ++i){
        std::cout << DEG(JointValuesInRad[i]) << ", ";
    }
    std::cout << "]"<<std::endl;

//    std::string ss;
//    std::cout << "Press a key to continue...\n";
//    std::cin >> ss;

    for (i = 0; i < NUMBER_OF_JOINTS; i++)
    {
        IP->CurrentPosition->VecData[i] = (double)DEG(JointValuesInRad[i]);
        //if(i == 1)
          //  IP->TargetPosition->VecData[i] = double(TargetJointValuesInDeg[i] - 90.0);
        //else
        IP->TargetPosition->VecData[i] = (double)TargetJointValuesInDeg[i];

        IP->MaxVelocity->VecData	 [i] = (double)10.0; // DO NOT INCREASE IN T1-T2 MODES
        IP->MaxAcceleration->VecData [i] = (double)20.0; // DO NOT INCREASE IN T1-T2 MODES
        IP->SelectionVector->VecData [i] = true;
    }

    ResultValue	= TypeIRML::RML_WORKING;

    while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED)){
        FRI->WaitForKRCTick();

        ResultValue	= RML->GetNextMotionState_Position(*IP,	OP);

        if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED)){
            printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
        }

        for (i = 0; i < NUMBER_OF_JOINTS; i++){
            JointValuesInRad[i]	= RAD((float)(OP->NewPosition->VecData[i]));
        }

        FRI->SetCommandedJointPositions(JointValuesInRad);

        *(IP->CurrentPosition) = *(OP->NewPosition);
        *(IP->CurrentVelocity) = *(OP->NewVelocity);
    }

    std::cout << "Position reached..." << std::endl;

    delete	RML;
    delete	IP;
    delete	OP;

    return 0;
}


int RunJointTrajectory(FastResearchInterface *FRI, float *TargetJointValuesInRad)
{
    unsigned int i = 0;

    int	ResultValue	= 0;

    float JointValuesInRad[NUMBER_OF_JOINTS];
    memset(JointValuesInRad, 0x0, NUMBER_OF_JOINTS * sizeof(float));

    double CycleTime = 0.002;

    TypeIRML *RML =	NULL;

    TypeIRMLInputParameters	*IP	= NULL;

    TypeIRMLOutputParameters *OP = NULL;

    RML	= new TypeIRML(NUMBER_OF_JOINTS, CycleTime);

    IP = new TypeIRMLInputParameters(NUMBER_OF_JOINTS);

    OP = new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

    if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
    {
        printf("Program is going to stop the robot.\n");
        FRI->StopRobot();

        FRI->GetMeasuredJointPositions(JointValuesInRad);
        FRI->SetCommandedJointPositions(JointValuesInRad);

        printf("Restarting the joint position control scheme.\n");
        ResultValue	=	FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);

        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            printf("An error occurred during starting up the robot...\n");
            delete	RML;
            delete	IP;
            delete	OP;

            return -1;
        }
    }

    FRI->GetMeasuredJointPositions(JointValuesInRad);

    for (i = 0; i < NUMBER_OF_JOINTS; i++)
    {
        IP->CurrentPosition->VecData[i] = (double)DEG(JointValuesInRad[i]);

        IP->TargetPosition->VecData[i] = (double)DEG(TargetJointValuesInRad[i]);

        IP->MaxVelocity->VecData	 [i] = (double)10.0; // DO NOT INCREASE IN T1-T2 MODES
        IP->MaxAcceleration->VecData [i] = (double)20.0; // DO NOT INCREASE IN T1-T2 MODES
        IP->SelectionVector->VecData [i] = true;
    }

    ResultValue	= TypeIRML::RML_WORKING;

    while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
    {
        FRI->WaitForKRCTick();

        ResultValue	= RML->GetNextMotionState_Position(*IP,	OP);

        if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
        {
            printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
        }

        for (i = 0; i < NUMBER_OF_JOINTS; i++)
        {
            JointValuesInRad[i]	= RAD((float)(OP->NewPosition->VecData[i]));
        }

        FRI->SetCommandedJointPositions(JointValuesInRad);

        *(IP->CurrentPosition) = *(OP->NewPosition);
        *(IP->CurrentVelocity) = *(OP->NewVelocity);
    }

    std::cout << "Position reached..." << std::endl;

    delete	RML;
    delete	IP;
    delete	OP;

    return 0;
}

int RunCartesianTrajectory(FastResearchInterface *FRI, float *TargetJointValuesInRad)
{
    unsigned int i = 0;

    int	ResultValue	= 0;

    float JointValuesInRad[NUMBER_OF_JOINTS];
    memset(JointValuesInRad, 0x0, NUMBER_OF_JOINTS * sizeof(float));

    double CycleTime = 0.002;

    TypeIRML *RML =	NULL;

    TypeIRMLInputParameters	*IP	= NULL;

    TypeIRMLOutputParameters *OP = NULL;

    RML	= new TypeIRML(NUMBER_OF_JOINTS, CycleTime);

    IP = new TypeIRMLInputParameters(NUMBER_OF_JOINTS);

    OP = new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

    if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
    {
        printf("Program is going to stop the robot.\n");
        FRI->StopRobot();

        FRI->GetMeasuredJointPositions(JointValuesInRad);
        FRI->SetCommandedJointPositions(JointValuesInRad);

        printf("Restarting the joint position control scheme.\n");
        ResultValue	=	FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);

        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            printf("An error occurred during starting up the robot...\n");
            delete	RML;
            delete	IP;
            delete	OP;

            return -1;
        }
    }

    FRI->GetMeasuredJointPositions(JointValuesInRad);

    for (i = 0; i < NUMBER_OF_JOINTS; i++)
    {
        IP->CurrentPosition->VecData[i] = (double)DEG(JointValuesInRad[i]);

        IP->TargetPosition->VecData[i] = (double)DEG(TargetJointValuesInRad[i]);

        IP->MaxVelocity->VecData	 [i] = (double)10.0; // DO NOT INCREASE IN T1-T2 MODES
        IP->MaxAcceleration->VecData [i] = (double)20.0; // DO NOT INCREASE IN T1-T2 MODES
        IP->SelectionVector->VecData [i] = true;
    }

    ResultValue	= TypeIRML::RML_WORKING;

    while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
    {
        FRI->WaitForKRCTick();

        ResultValue	= RML->GetNextMotionState_Position(*IP,	OP);

        if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
        {
            printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
        }

        for (i = 0; i < NUMBER_OF_JOINTS; i++)
        {
            JointValuesInRad[i]	= RAD((float)(OP->NewPosition->VecData[i]));
        }

        FRI->SetCommandedJointPositions(JointValuesInRad);

        *(IP->CurrentPosition) = *(OP->NewPosition);
        *(IP->CurrentVelocity) = *(OP->NewVelocity);
    }

    std::cout << "Position reached..." << std::endl;

    delete	RML;
    delete	IP;
    delete	OP;

    return 0;
}
