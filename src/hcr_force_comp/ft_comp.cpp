#include "ft_comp.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;


ft_comp::ft_comp(double mass, double* com): m_mass(mass)
{
    memcpy(this->com, com, 3*sizeof(double));
}

void ft_comp::setMass(double mass){
    m_mass=mass;
}

void ft_comp::compensate(double *pose, double *ft_measured, double *ft_final) {

    Map<Matrix<double, 3, 4, RowMajor>> Pose(pose);

    Eigen::Matrix<double, 3,3> R;
    R = Pose.block<3,3>(0,0);
    //R.transposeInPlace();

    Eigen::Map<Vector3d> v_gravity(gravity, 3);
    Eigen::Vector3d gravity_comp_1(3);
    gravity_comp_1<< m_mass*R.transpose()*v_gravity;

    Map<Vector3d> v_com(com, 3);
    Vector3d gravity_comp_2(3);
    gravity_comp_2<< (m_mass*v_com).cross(R.transpose()*v_gravity);

    VectorXd v_g_comp (gravity_comp_1.size() + gravity_comp_2.size());
    v_g_comp << gravity_comp_1, gravity_comp_2;

    Map<Matrix<double,6,1>> v_measured(ft_measured);
    Matrix<double, 6,1> v_final;
    Map<Matrix<double,6,1>> v_bias(bias);

    v_final << v_measured + v_g_comp + v_bias;


    //cout << "v_measured:" << endl << v_measured.block<3,1>(0,0) << endl;
    //cout << "gravity_comp: " << endl << v_g_comp << endl;
    //cout << "v_bias: " << endl << v_bias.block<3,1>(0,0) << endl;
    //cout << "v_final: " << endl << v_final.block<3,1>(0,0) << endl;


    memcpy(ft_final, v_final.data(), 6*sizeof(double));

    }



void ft_comp::reset(double* ft_measured){

    memcpy(bias, ft_measured, 6*sizeof(double));

    for(int i=0; i<6; i++)
        bias[i] = -bias[i];
    //Map<Matrix<double,6,1>> v_bias(bias);
    //cout << "ft_measured:" << endl << v_bias << endl;
    bias [0] += m_mass*gravity[0];
    bias[1] += m_mass*gravity[1];
    bias[2] += m_mass*gravity[2];
    bias[3] += m_mass*com[1]*gravity[2];    // cross product with gravity vector
    bias[4] -= m_mass*com[0]*gravity[2];    // cross product with gravity vector
    bias[5] += 0;                          // cross product with gravity vector

    /*
    bias[0] -= m_mass*gravity[0];
    bias[1] -= m_mass*gravity[1];
    bias[2] -= m_mass*gravity[2];
    bias[3] -= m_mass*com[1]*gravity[2];
    bias[4] += m_mass*com[0]*gravity[2];
    bias[5] += 0;
    */









}

