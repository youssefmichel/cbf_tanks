#include "PassiveControl.h"
#include "iostream"
#include "eigen3/Eigen/Dense"
#include "cmath"


using Eigen::MatrixXd;
using namespace Eigen;

using namespace std;
using namespace special_math_functions;
using namespace StiffnessProfiles;

ClosedLoopConfiguration::ClosedLoopConfiguration(Mat L_, int M_):
    L(L_),
    M(M_)

{
    K_o = 40.0;
    D_o = 15;
//    K_1 = 200;
//    D_1 = 20;
    K_1 = 2000;
    D_1 = 60;




}

Vec ClosedLoopConfiguration::GetControlTorque(Vec x_dot_d, Vec x_dot, Mat Jac, realtype x_c, Vec x)
{
    Vec F(3);
    Mat Q = FindDampingBasis(x_dot_d);
    Mat D = Q * L * Q.transpose();
    if (M == 3){
        F = - D * x_dot + x_dot_d;
    }
    else{
        F(0) = K_1 * (x_c - x(0)) - D_1 * x_dot(0);
        Vec F_temp = - D * x_dot.block(1,0,2,1) + L(0,0) * x_dot_d;
        //      Vec F_temp = - D * x_dot.block(1,0,2,1) + 1 * x_dot_d;
        F(1) = F_temp(0);
        F(2) = F_temp(1);
    }


    Mat Jac_t = Jac.block(0, 0, 3, 7);
    Vec tau_x = Jac_t.transpose() * F;
    return F ;
}

Vec ClosedLoopConfiguration::ComputeOrientationForce(Mat  R_a, Mat R_d,Mat Jac,Vec q_dot) {


    // Implementation Based on Franka Emika
    //Angle axis constructor accepts only Matrix3d type
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

    Eigen::Matrix3d R_ad=R_aa.transpose()*R_dd ;
    Eigen::Quaterniond orientation_ad(R_ad);
    error << orientation_ad.x(), orientation_ad.y(), orientation_ad.z();
    error<< R_a * error;


    Mat Jac_o = Jac.block(3, 0, 3, 7);

    Vec F_o = K_o*error -D_o*( (Jac_o*q_dot))   ;

    Vec tau_o=(Jac_o.transpose() * F_o) ;

    return  tau_o;

}

Mat ClosedLoopConfiguration::FindDampingBasis(Vec x)
{
    if (M == 2){
        Vec y(2);
        y << 1,
                -x(0)/(x(1)+eps());
        Mat B = Mat::Zero(2,2);
        B.col(0) = x/x.norm();
        B.col(1) = y/y.norm();
        return B;
    } else {
        Vector3d x_ = x;
        Vector3d y(1,1,(-x(0)-x(1))/(x(2)+eps()));
        Vector3d z;
        z = x_.cross(y);
        Mat B = Mat::Zero(3,3);
        B.col(0) = x/x.norm();
        B.col(1) = y/y.norm();
        B.col(2) = z/z.norm();
        return B;
    }
}

//----------------------------------Energy Tank and Stablization Implementation----------------------
PassiveClosedLoopConfiguration::PassiveClosedLoopConfiguration(Mat L_,int M_,Vec x_0,Vec x_att_):
    ClosedLoopConfiguration(L_,M_){

    tau_max=15 ;
    K_const=150 ;

    //For the ln based stiff Potential
    thresh_ln=0.05 ;
    K_const_ln=280  ;


    wind_act=4800 ; //4800
    x_att=x_att_ ;
    realtype act=Smooth_Activation_Exp(x_0-x_att) ;
    


    stepness = x_att_ ; //0.0006

    stepness <<0.0006,0.0006 ;


    ko=0*x_att_ ;
    ko<< 0.57, 0.53;
    ko<< 0.59, 0.63;
    ko<< 0.34, 0.0001;
    ko<< 0.48, 0.45;
    ko<< 0.48, 0.48;
   // ko<< 0.58, 0.58;



    beta=1 ;
    gamma=beta ;
    s_tank_init=30 ;
    s_tank_max=act*s_tank_init ;
    s_tank=0.79*s_tank_max ;
    ds=0.1*s_tank_max ;
    Pow_VSDS=0 ;

}


realtype PassiveClosedLoopConfiguration::Smooth_Activation_Exp(Vec x){

    return (1-exp(-wind_act*x.dot(x))) ;
}

realtype PassiveClosedLoopConfiguration::Get_Pow_VSDS_aka_z(){
    return Pow_VSDS ;
}

realtype PassiveClosedLoopConfiguration::Energy_Tank(realtype Pow_VSDS,realtype Pow_Damp, Vec x, realtype dt){


    realtype act=Smooth_Activation_Exp(x-x_att) ;
    s_tank_max= act*s_tank_init ;
    realtype s_tank_min =0.5 ;
    realtype alpha=1 ;

    if(s_tank<s_tank_max){
        alpha=0.99 ;
    }
    else{
        alpha=0 ;
    }
    beta=1 ;
    if(s_tank>s_tank_max && Pow_VSDS<0){
        beta=0 ;
    }

    else if(s_tank<s_tank_min && Pow_VSDS>0){
        beta=0 ;
    }
    if(s_tank>s_tank_max){

        s_tank = s_tank_max;
    }
    else{
        alpha = min(0.99, alpha);
        realtype sdot = alpha*Pow_Damp - beta*Pow_VSDS;

        s_tank = s_tank + sdot*0.01;
    }
    //gamma=beta ;
    return gamma ;
}



realtype PassiveClosedLoopConfiguration::Get_Tank_gamma(){
    return gamma ;
}

realtype PassiveClosedLoopConfiguration::Get_Tank_beta(){
    return beta ;
}

realtype PassiveClosedLoopConfiguration::Get_Tank_Energy(){
    return s_tank ;
}


Vec PassiveClosedLoopConfiguration::Stiff_Potential_Matteo_Consv(Vec x_err) {

    int size=max(x_err.rows(),x_err.cols()) ;
    Vec center = Vec::Zero(size) ;
    Vec F = Vec::Zero(size) ;

   // ko<< 0.45, 0.85 ;

    realtype tau_min = 0.0;


    for (int i=0 ; i< size ; i++){

        F(i)=-(ko(i)/stepness(i))*x_err(i)*exp(-pow(x_err(i)-center(i),2) / (2*stepness(i))) - (2*tau_min*x_err(i)) ;



    }

    if(abs(F(0))>1 && abs(F(1))>1){

        cout<<"FLAGGGG "<<endl ;
        cout<<"FF::: "<<F<<endl ;
           }
    else{
      // F=F*0 ;

    }


    return F ;
}


Vec PassiveClosedLoopConfiguration::GetControlTorque_Passive(Vec x_dot_d, Vec x_dot, realtype x_c, Vec x,realtype dt)
{
    Vec F=Vec::Zero(3);
    Mat Q = FindDampingBasis(x_dot_d.head(2));
    Mat D = Q * L * Q.transpose();
    // Mat D=L ;
    if (M == 3){

        Vec F_VSDS= x_dot_d ;
        Vec F_Damp= - D * x_dot ;

        realtype Pow_Damp= x_dot.transpose()*F_Damp  ;
        realtype Pow_VSDS= x_dot.transpose() *F_VSDS;

        Vec F_constPot=Stiff_Potential_Matteo_Consv(x.tail(2)-x_att) ;
        realtype beta=Energy_Tank_smooth(Pow_VSDS,Pow_Damp, x,  dt) ;
        Vec F_tmp= beta*F_VSDS +F_Damp +F_constPot ;
        F=F_tmp ;

    }
    else{

        F(0) = K_1 * (x_c - x(0)) - D_1 * x_dot(0);

        // Vec F_constPot=ConstStiffPotential(x_att-x.tail(2));
        // Vec F_constPot=ConstStiffPotential_ln(x.tail(2)-x_att) ;
        // Vec F_constPot=Stiff_Potential_Matteo(x_att-x.tail(2),x_dot.tail(2)) ;
        Vec F_constPot=Stiff_Potential_Matteo_Consv(x.tail(2)-x_att) ;
        Vec F_VSDS=  1.0*Smooth_Activation_Exp( x_att-x.tail(2))*x_dot_d.head(2) ;

        Vec F_Damp= x_dot_d.tail(2) ;
        Vec F_Damp_const=Vec::Zero(2) ;

        for (int i=0 ; i< 2 ; i++){

            realtype err=x(i+1)-x_att(i) ;

            realtype k_gain=(ko(i)*err*err*exp(-pow(err,2)/(2*stepness(i))))/pow(stepness(i),2) - (ko(i)*exp(-pow(err,2)/(2*stepness(i))))/stepness(i) ;
            realtype d_gain=abs(70/0.0135*err*exp(-pow(err,2) / (2*stepness(i))) ) ;

            //d_gain =2*0.6*sqrt(abs(k_gain)) ;
            d_gain=d_gain*0.3 ;

            F_Damp_const(i)=-d_gain *x_dot(i+1) ;
        }

        realtype Pow_Damp= -x_dot.tail(2).transpose()  *   F_Damp  ; // Added - since should be positive power
        Pow_VSDS=  x_dot.tail(2).transpose()   *  F_VSDS ;

        Energy_Tank_smooth(Pow_VSDS,Pow_Damp, x.tail(2),  dt) ; // gamma will get updated
    //  Energy_Tank(Pow_VSDS,Pow_Damp, x.tail(2),  dt) ;

        Vec  F_temp(2) ;

        cout<<"FF::: "<<F_constPot<<endl ;
        F_temp= gamma*F_VSDS +F_Damp + 1*F_constPot + 0*F_Damp_const ;
       //  F_temp= gamma*F_VSDS +F_Damp  ;


        cout<<"F Tmp"<<F_temp<<endl ;

    // F_temp= F_VSDS +F_Damp ;


        F(1) = F_temp(0);
        F(2) = F_temp(1);

    }


    return F ;
}

realtype PassiveClosedLoopConfiguration::Energy_Tank_smooth(realtype Pow_VSDS,realtype Pow_Damp, Vec x, realtype dt){

    realtype act=Smooth_Activation_Exp(x-x_att) ;
    s_tank_max= act*s_tank_init ;
    ds=0.1*s_tank_max ;
    ds=2 ;
    realtype s_tank_min =0 ;
    realtype z=Pow_VSDS ;
    realtype dz=0.1 ;

    // gamma = smooth_rise_2d(z,  s_tank, 0, dz, 0, ds);

    if(abs(z)<0.001){
        z=0 ;
    }
    if(s_tank >s_tank_max){

        cout<<"Tank !!!!  and Z: "<<z<<endl ;
    }

    beta  = smooth_rise_fall_2d(z,s_tank, 0.000, 0.000, dz, s_tank_min, s_tank_max, ds);
    beta=max(beta,0.0) ;
    gamma = smooth_fall_gamma(z,-0.1, 0, beta ) ;
    realtype alpha = smooth_rise_fall(s_tank, 0, ds, s_tank_max-ds, s_tank_max);


//    if(s_tank>s_tank_max){
//        s_tank = s_tank_max;
//    }
//    else {
//        alpha = min(0.99, alpha);
//        realtype sdot = alpha*Pow_Damp - beta*z;
//        s_tank = s_tank + sdot*dt;
//    }

    //New Formulation
    alpha = min(0.99, alpha);
    realtype sdot = alpha*Pow_Damp - beta*z  -(1.05-act)*s_tank ;
    s_tank = s_tank + sdot*dt ;



    return gamma ;



}


realtype PassiveClosedLoopConfiguration::smooth_fall(realtype val,realtype hi, realtype lo){

    realtype smooth_value=1 ;

    if(hi>lo) {
        cout<<"Error in Smooth Fall"<<endl ;
        smooth_value = -1;
    }

    if(val>=lo){
        smooth_value = 0; }
    else if(val<hi){
        smooth_value = 1;}
    else{
        realtype T = 2*(lo-hi);
        smooth_value = 0.5+0.5*sin(2*pi*(val-lo)/T - pi*0.5);
    }
    return smooth_value ;

}

realtype PassiveClosedLoopConfiguration::smooth_rise(realtype val, realtype lo,realtype hi){
    realtype smooth_value=1 ;
    if(lo>hi){
        cout<<"Error in Smooth Rise"<<endl ;
        smooth_value = -1;

    }

    if(val>=hi){
        smooth_value = 1; }
    else if(val<lo){
        smooth_value = 0;}
    else{
        realtype T = 2*(hi-lo);
        smooth_value = 0.5+0.5*sin(2*pi*(val-lo)/T - pi*0.5);
    }
    return smooth_value ;
}

realtype PassiveClosedLoopConfiguration::smooth_rise_2d(realtype x, realtype y, realtype xlo, realtype dx, realtype ylo, realtype dy){
    realtype h1 = smooth_rise(x, xlo, xlo+dx);
    realtype h2 = smooth_fall(y, ylo, ylo+dy);

    realtype h = 1 - h1*h2;
    return h ;
}

realtype PassiveClosedLoopConfiguration::smooth_rise_fall_2d( realtype x, realtype y, realtype xlo, realtype xhi,realtype dx, realtype ylo, realtype yhi, realtype dy){
    realtype h1_1 = smooth_rise(x, xlo-dx, xlo);
    realtype h1_2 = smooth_fall(y, ylo, ylo+dy);
    realtype h1 = h1_1*h1_2;

    realtype h2_1 = smooth_fall(x, xhi, xhi+dx);
    realtype h2_2 = smooth_rise(y, yhi-dy, yhi);
    realtype h2 = h2_1*h2_2;

    realtype h = 1 - h1 - h2;
    return h ;
}

realtype PassiveClosedLoopConfiguration::smooth_rise_fall(realtype val, realtype l0,realtype l1,realtype h1, realtype h0){
    realtype hr = smooth_rise( val,  l0,  l1);
    realtype hf = smooth_fall( val, h1, h0);
    realtype h = hr*hf;
    return h ;
}


realtype PassiveClosedLoopConfiguration::smooth_fall_gamma(realtype val,realtype hi, realtype lo,realtype beta ){

    realtype smooth_value=1 ;

    if(hi>lo) {
        cout<<"Error in Smooth Fall"<<endl ;
        smooth_value = -1;
    }
    realtype scale=(1-beta) ;

    if(val>=lo){
        smooth_value = beta; }
    else if(val<hi){
        smooth_value = 1;}
    else{
        realtype T = 2*(lo-hi);
        smooth_value = (0.5+0.5*sin(2*pi*(val-lo)/T - pi*0.5))*scale + beta;
    }
    return smooth_value ;

}


