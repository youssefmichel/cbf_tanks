


#include "utility_fri.h"

int startGravityCompensation(FastResearchInterface *FRI, float duration) {

    float  JointStiffnessValues[LBR_MNJ],
            EstimatedExternalCartForcesTorques[FRI_CART_VEC],
            EstimatedExternalJointTorques[LBR_MNJ],
            currentCartPose[FRI_CART_FRM_DIM] ,
            currentJointPosition[LWR_JNT_NUM];

     if(startJointImpedanceCtrl(FRI, currentJointPosition)==0){

         printf("Gravity compensation Started \n");
         // Set stiffness
         for(int i = 0; i < LWR_JNT_NUM; i++){
             JointStiffnessValues[i] = (float)MIN_STIFFNESS; // max stiffness 0-2 -> 2000.0, max 3-5 200.0
         }

         FRI->SetCommandedJointStiffness(JointStiffnessValues);
         int it_= 0 ;

         int LoopValue = int(duration / FRI->GetFRICycleTime());
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

             FRI->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesTorques);
             FRI->GetEstimatedExternalJointTorques(EstimatedExternalJointTorques);

             it_++;

         }
         cout<<"Joint Angles: \n "  << float_To_Vec(currentJointPosition,7) *(180/PI)	<<endl ;
         cout<<"Cartesian Position: \n "  << GetTranslation(currentCartPose)<<endl ;



     }


}

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

vector< std::vector <float> > KukaMoveCartesianMinJerk(FastResearchInterface *FRI, float tot_time, float dt, Vec x_df) {
    printf("Moving to Desired Cartesian Position with a minimum Jerk Trajectorty  \n");
    float CartStiffnessValues[FRI_CART_VEC],  CartDampingValues[FRI_CART_VEC], CommandedForcesAndTorques[FRI_CART_VEC] ;
    float currentCartPose[FRI_CART_FRM_DIM], commCartPose[FRI_CART_FRM_DIM],currentjointpos[7];
    FRI->GetMeasuredCartPose(currentCartPose);


    for(int i=0; i<FRI_CART_FRM_DIM; i++){
        commCartPose[i]=currentCartPose[i] ;
    }
    std::vector< std::vector <float> > X_pos_Vec;

    MinJerk *min_jerk_profile;
    min_jerk_profile = new MinJerk(dt,tot_time, GetTranslation(currentCartPose),x_df );


    for (int i = 0; i < 6; i++)
    {
        if (i==0 || i==1 || i==2){
            CartStiffnessValues[i] = (float)2500;
            CartDampingValues[i]=(float)0.7;
        }
        else{
            CartStiffnessValues[i] =(float)200.0 ;
            CartDampingValues[i]=(float)0.7;
        }
        CommandedForcesAndTorques	[i]	=	(float)0.0;
    }

    CartStiffnessValues[1] = (float)2000.0;

    FRI->SetCommandedCartStiffness(CartStiffnessValues);
    FRI->SetCommandedCartDamping(CartDampingValues);
    FRI->SetCommandedCartForcesAndTorques( CommandedForcesAndTorques);
    FRI->SetCommandedCartPose(commCartPose);

    Vec err_x = Vec::Ones(3) ; realtype tol=0.01 ; realtype Max_time=55 ;  realtype t=0 ; Vec x=GetTranslation(currentCartPose);
    Vec x_d=GetTranslation(currentCartPose) ; int wv_index=0 ; Mat R=GetRotationMatrix(currentCartPose) ;
    Mat R_d=R ;
    R_d << -1,0,0,
            0,1,0,
            0,0,-1;
    R_d=R ;

    //
    int done_g=1 ;
    while ( (FRI->IsMachineOK()) && err_x.norm()>tol && t<Max_time  ){

        FRI->WaitForKRCTick();
        X_pos_Vec.push_back( std::vector <float>() );
        min_jerk_profile->Update(t);
        x_d=min_jerk_profile->GetDesiredPos() ;
        commCartPose[3]=x_d(0);commCartPose[7]=x_d(1);commCartPose[11]=x_d(2);
        if (dhdKbHit() && dhdKbGet()=='q') done_g = -1;


        FRI->GetMeasuredCartPose(currentCartPose);
        for (int i = 0; i < FRI_CART_VEC; i++){
            if(i<3){

            }

            else{

                CartStiffnessValues[i] =200 ;
            }

        }

        SetDesiredPoseFRI(R_d,x_d,commCartPose);
        FRI->SetCommandedCartStiffness(CartStiffnessValues);


        FRI->GetMeasuredCartPose(currentCartPose);
        FRI->GetMeasuredJointPositions(currentjointpos) ;
        Vec q=float_To_Vec(currentjointpos,7) ;
        x=GetTranslation(currentCartPose) ; R=GetRotationMatrix(currentCartPose) ;


        err_x=x-x_df ;
        FRI->SetCommandedCartPose(commCartPose);
        for(int j=0; j<3; ++j){
            X_pos_Vec[wv_index].push_back(x(j));
        }

        for(int j=0; j<3; ++j){
            X_pos_Vec[wv_index].push_back(x_d(j));
        }
        for(int j=0; j<7; ++j){
            X_pos_Vec[wv_index].push_back(q(j));
        }


        wv_index++ ;
        t+=dt ;

    }

    FRI->GetMeasuredCartPose(currentCartPose);
    FRI->SetCommandedCartPose(currentCartPose);
    std::cout << "Cartesian Position reached..." << std::endl;
    cout<<"Current Cartesisian position: \n "  << GetTranslation(currentCartPose)	<<endl ;
    cout<<"Desired Final GOal: \n "  << x_df	<<endl ;

    return X_pos_Vec ;

}




int MoveCartesian_MinJerk_FullPose(FastResearchInterface *FRI, float tot_time, float dt, Vec x_df, Quaterniond quat_df) {

    // Moves to a full 3D Pose (position +orientation)

    printf("Moving to Desired Cartesian Position and Orientation with a minimum Jerk Trajectorty  \n");
    float CartStiffnessValues[FRI_CART_VEC],  CartDampingValues[FRI_CART_VEC], CommandedForcesAndTorques[FRI_CART_VEC] ;
    float currentCartPose[FRI_CART_FRM_DIM], commCartPose[FRI_CART_FRM_DIM],currentjointpos[7];
    FRI->GetMeasuredCartPose(currentCartPose);
    FRI->GetMeasuredCartPose(commCartPose);
    startCartImpedanceCtrl(FRI,currentCartPose) ;

    MinJerk *min_jerk_profile;
    min_jerk_profile = new MinJerk(dt,tot_time, GetTranslation(currentCartPose),x_df );


    for (int i = 0; i < 6; i++)
    {
        if (i<3){
            CartStiffnessValues[i] = (float)2000;
            CartDampingValues[i]=(float)0.7;
        }
        else{
            CartStiffnessValues[i] =(float)200.0 ;
            CartDampingValues[i]=(float)0.7;
        }
        CommandedForcesAndTorques	[i]	=	(float)0.0;
    }



    FRI->SetCommandedCartStiffness(CartStiffnessValues);
    FRI->SetCommandedCartDamping(CartDampingValues);
    FRI->SetCommandedCartForcesAndTorques( CommandedForcesAndTorques);
    FRI->SetCommandedCartPose(commCartPose);

    Vec err_full = Vec::Ones(FRI_CART_VEC) ; realtype tol=0.01 ; realtype Max_time=20 ;  realtype t=0 ; Vec x=GetTranslation(currentCartPose);
    Vec x_d= GetTranslation(currentCartPose) ;
    Matrix3d R=  GetRotationMatrix(currentCartPose) ;
    Quaterniond q_init(R) ;
    Matrix3d R_df =quat_df.normalized().toRotationMatrix() ;
    int done_g=1 ;

    while ( (FRI->IsMachineOK()) && err_full.norm()>tol && t<Max_time  && done_g){


         cout<<"error" <<err_full.norm()<<endl ;

        FRI->WaitForKRCTick();
        if (dhdKbHit() && dhdKbGet()=='q') done_g = -1;

       min_jerk_profile->Update(t);
       x_d=min_jerk_profile->GetDesiredPos() ;

       // x_d=x_df ;
        R=  GetRotationMatrix(currentCartPose) ;
        Quaterniond q_des = q_init.slerp(t/Max_time, quat_df);
        Matrix3d R_d = q_des.normalized().toRotationMatrix() ;



        for (int i = 3; i < FRI_CART_VEC; i++){

                  // Use Smoothly rising orientation stiffness .  @TODO: Replace with Slerp
                //  CartStiffnessValues[i] = GeneralFunction::smooth_transition_rising(t, 9.0, 0, 50);
                  CartStiffnessValues[i] = 200;
                  CartDampingValues[i]=(float)0.7;
        }

        SetDesiredPoseFRI(R_d,x_d,commCartPose);
        FRI->SetCommandedCartStiffness(CartStiffnessValues);
        FRI->SetCommandedCartDamping(CartDampingValues);
        FRI->SetCommandedCartPose(commCartPose);

        FRI->GetMeasuredCartPose(currentCartPose);
        FRI->GetMeasuredJointPositions(currentjointpos) ;

        x=GetTranslation(currentCartPose) ; R=GetRotationMatrix(currentCartPose) ;


       err_full.head(3)=x-x_df ;
       err_full.tail(3)=QuartOrientErr(R,R_df) ;

        t+=dt ;

    }

    FRI->GetMeasuredCartPose(currentCartPose);
    FRI->SetCommandedCartPose(currentCartPose);
    Matrix3d Rot_3d= GetRotationMatrix(currentCartPose) ;
    Quaterniond Quat_curr(Rot_3d) ;


    if(err_full.norm()<tol )
    std::cout << "Desired Position reached..." << std::endl;

    cout<<"Current Translational pos: \n "  << GetTranslation(currentCartPose)	<<endl ;
    cout<<"Desired Final GOal: \n "  << x_df	<<endl ;

    cout<<"Current Rotation: \n "  << GetRotationMatrix(currentCartPose)	<<endl ;
   // cout<<"Desired Final Rotation: \n "  << R_	<<endl ;

    cout<<"Current Quaternion: \n "  << Quat_curr.w()<< Quat_curr.vec()	<<endl ;
    cout<<"Desired Final Quaternion: \n "  << quat_df.w()<<quat_df.vec()<<endl ;

    return 1 ;

}





int MoveCartesian_Trajectory_FullPose(FastResearchInterface *FRI, string des_traj_file,string Save_Data_dir) {

    // Moves to a full 3d Pose (position +orientation) trajectory specified in file

    printf("Moving to Desired Cartesian Position and Orientation with a Desired Trajectorty  \n");
    float CartStiffnessValues[FRI_CART_VEC],  CartDampingValues[FRI_CART_VEC], CommandedForcesAndTorques[FRI_CART_VEC] ;
    float currentCartPose[FRI_CART_FRM_DIM], commCartPose[FRI_CART_FRM_DIM],currentjointpos[7];

    float   EstimatedExternalCartForcesTorques[FRI_CART_VEC],
            EstimatedExternalJointTorques[LBR_MNJ];


    std::vector< std::vector <float> > x_rob_Vector;
    std::vector< std::vector <float> > x_des_Vector;
    std::vector< std::vector <float> > F_ext_Vector;
    std::vector< std::vector <float> > tau_ext_Vector;

    std::string x_rob_file =Save_Data_dir+"x_rob"+".txt";
    std::string x_des_file =Save_Data_dir+"x_des"+".txt";
    std::string F_ext_file =Save_Data_dir+"F_ext"+".txt";
    std::string tau_ext_file =Save_Data_dir+"tau_ext"+".txt";

    FRI->GetMeasuredCartPose(currentCartPose);
    FRI->GetMeasuredCartPose(commCartPose);
    startCartImpedanceCtrl(FRI,currentCartPose) ;

   std::vector< std::vector <float> > motion_;
   loadVectorMatrixFromFile(des_traj_file, 7, motion_);
   int dim_traj = (int)motion_.size();


    for (int i = 0; i < 6; i++)
    {
        if (i<3){
            CartStiffnessValues[i] = (float)700;
            CartDampingValues[i]=(float)0.7;
        }
        else{
            CartStiffnessValues[i] =(float)80.0 ;
            CartDampingValues[i]=(float)0.7;
        }
         CartStiffnessValues[2] = (float)2000;
        CommandedForcesAndTorques	[i]	=	(float)0.0;
    }



    FRI->SetCommandedCartStiffness(CartStiffnessValues);
    FRI->SetCommandedCartDamping(CartDampingValues);
    FRI->SetCommandedCartForcesAndTorques( CommandedForcesAndTorques);
    FRI->SetCommandedCartPose(commCartPose);

    realtype tol=0.01 ; realtype Max_time=30 ;  realtype t=0 ; Vec x=GetTranslation(currentCartPose);
    Vec x_d= GetTranslation(currentCartPose) ;
    Matrix3d R=  GetRotationMatrix(currentCartPose) ;

    Quaterniond q_des(R) ;
    realtype dt= FRI->GetFRICycleTime()  ;

    int done_g=1 ;
    int file_counter=0  ;
    int CycleCounter= 0 ;

    while ( (FRI->IsMachineOK()) && t<Max_time  && done_g){



        FRI->WaitForKRCTick();
        if (dhdKbHit() && dhdKbGet()=='q') done_g = -1;

        FRI->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesTorques);
        FRI->GetEstimatedExternalJointTorques(EstimatedExternalJointTorques);



        x_d(0)=motion_[file_counter][0];
        x_d(1)=motion_[file_counter][1];
     //   x_d(2)=motion_[file_counter][2];
        q_des.w()=motion_[file_counter][3];
        q_des.x()=motion_[file_counter][4];
        q_des.y()=motion_[file_counter][5];
        q_des.z()=motion_[file_counter][6];

        if(file_counter<dim_traj-1){
            file_counter ++ ;
        }


        R=  GetRotationMatrix(currentCartPose) ;
        Matrix3d R_d = q_des.normalized().toRotationMatrix() ;

        SetDesiredPoseFRI(R_d,x_d,commCartPose);

        FRI->SetCommandedCartStiffness(CartStiffnessValues);
        FRI->SetCommandedCartDamping(CartDampingValues);
        FRI->SetCommandedCartPose(commCartPose);

        FRI->GetMeasuredCartPose(currentCartPose);
        FRI->GetMeasuredJointPositions(currentjointpos) ;

        x=GetTranslation(currentCartPose) ; R=GetRotationMatrix(currentCartPose) ;
        Quaterniond q_curr(R) ;





        x_rob_Vector.push_back( std::vector <float>() );
        x_des_Vector.push_back( std::vector <float>() );
        F_ext_Vector.push_back( std::vector <float>() );
        tau_ext_Vector.push_back( std::vector <float>() );

        x_rob_Vector[CycleCounter].push_back(x(0));
        x_rob_Vector[CycleCounter].push_back(x(1));
        x_rob_Vector[CycleCounter].push_back(x(2));
        x_rob_Vector[CycleCounter].push_back(q_curr.w());
        x_rob_Vector[CycleCounter].push_back(q_curr.x());
        x_rob_Vector[CycleCounter].push_back(q_curr.y());
        x_rob_Vector[CycleCounter].push_back(q_curr.z());

        x_des_Vector[CycleCounter].push_back(x_d(0));
        x_des_Vector[CycleCounter].push_back(x_d(1));
        x_des_Vector[CycleCounter].push_back(x_d(2));
        x_des_Vector[CycleCounter].push_back(q_des.w());
        x_des_Vector[CycleCounter].push_back(q_des.x());
        x_des_Vector[CycleCounter].push_back(q_des.y());
        x_des_Vector[CycleCounter].push_back(q_des.z());

        for (int i=0;i <LWR_JNT_NUM ; i++ ){
            tau_ext_Vector[CycleCounter].push_back(EstimatedExternalJointTorques[i]);
        }
        for (int i=0;i <FRI_CART_VEC ; i++ ){
            F_ext_Vector[CycleCounter].push_back(EstimatedExternalCartForcesTorques[i]);
        }


        t+=dt ;
        CycleCounter ++ ;

    }

    saveVectorMatrixToFile(x_rob_file, x_rob_Vector);
    saveVectorMatrixToFile(x_des_file, x_des_Vector);
    saveVectorMatrixToFile(F_ext_file,F_ext_Vector);
    saveVectorMatrixToFile(tau_ext_file,tau_ext_Vector);


    return 1 ;

}



void SetDesiredPoseFRI(Mat Rot_d,Vec x_d,float *CartPose_d){

    int tot_count=0 ;

    for (int i=0;i<3;i++){

        int j=0 ;

        while(j<3){

            if ( !(tot_count==3 || tot_count==7|| tot_count==11 ) ){

                CartPose_d[tot_count]=Rot_d(i,j) ;

                j=j+1 ;

            }
            tot_count++ ;

        }



    }
    CartPose_d[3]=x_d(0) ;
    CartPose_d[7]=x_d(1) ;
    CartPose_d[11]=x_d(2) ;



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
    return 0;
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



float low_pass(float signal, float prev_filt, float cutt_off, float cycle_time){

    return   ( ( signal*cutt_off*cycle_time)+ prev_filt )/(1+ cutt_off*cycle_time) ;

}
Vec low_pass(Vec signal, Vec prev_filt, float cutt_off, float cycle_time){

    Vec out=Vec::Zero(signal.size()) ;
    for (int i=0;i<signal.size();i++){

        out(i)=low_pass(signal(i),prev_filt(i),cutt_off,cycle_time) ;
    }
    return out ;


}

//void Exit_Func(int sig){ // can be called asynchronously
//    Exit_flag=1 ;
//}





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



Mat ConvertJacobianToMat(float ** Jacobian){

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

Mat ConvertMassMatToMat(float ** Mass){

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



Vec float_To_Vec(float *inp,int size){

    Vec out(size) ;

    for (int i=0;i<size;i++){

        out(i)=inp[i] ;
    }
    return out ;

}

Mat Tool_To_WorldJacobian(Mat Jac_tool,Mat Rot){
    Mat Jac_world=Jac_tool ;
    Mat zeros=MatrixXd::Zero(3,3);
    Mat Trans(6,6) ;
    Trans<<Rot,zeros ,
            zeros,Rot ;
    Jac_world=Trans *Jac_tool ;

    return Jac_world ;



}

Vec QuartOrientErr(Mat R_a, Mat R_d){

    Eigen::Matrix3d R_aa=R_a;
    Eigen::Matrix3d R_dd = R_d;
    Vec error=Vec::Zero(3) ;


    Eigen::Quaterniond orientation_d(R_dd);
    Eigen::Quaterniond orientation(R_aa);

    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error<< R_a * error;
    return error ;
}
