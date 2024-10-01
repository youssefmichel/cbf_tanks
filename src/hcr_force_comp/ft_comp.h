#ifndef FT_COMP_H
#define FT_COMP_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;


class ft_comp
{
private:

    double com[3] = {-0.000793626, -0.00496703, 0.052861};
    //double bias[6] = {4.4999,15.9683,-30.1328,0.15018,0.0126887,-0.0174006};
    double bias[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
    double ft_measured[6];
    double gravity[3]={0,0, 9.81};
    double m_mass = 1.43583;
    double ft_final[6];
    VectorXd v_g_comp;

public:

    ft_comp(double mass, double* com);
    void setMass(double mass);
    void compensate(double *pose, double *ft_measured, double *ft_final);
    void reset(double* ft_measured);


};

#endif // FT_COMP_H
