
//
// Created by abudakka on 15/2/21.
//
#include "quatUtils.h"

namespace  QuatUtils  {


int quat_conjugate(double q1[], double q2[])
// Quaternion conjugation
{
    q2[0] = q1[0];
    for (int i = 1; i <= 3; i++)
        q2[i] = -q1[i];
    return 0;
}

int quat_mult(double q1[], double q2[], double q[])
// Quaternion multiplication
{
q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
for (int i = 1; i <= 3; i++)
    q[i] = q1[0] * q2[i] + q2[0] * q1[i];
    q[1] += q1[2] * q2[3] - q1[3] * q2[2];
    q[2] += q1[3] * q2[1] - q1[1] * q2[3];
    q[3] += q1[1] * q2[2] - q1[2] * q2[1];
    return 0;
}

int quat_exp(double omega[], double q[])
//Quaternion exponential
{
    double tmp;
    vnorm(omega,tmp);

    if (tmp > eps){
        q[0] = cos(tmp);
        double s = sin(tmp);
        for (int i = 0; i < 3; i++)
            q[i + 1] = s * omega[i] / tmp;
    }else{
        q[0] = 1.0;
        q[1] = q[2] = q[3] = 0.0;
    }
    return 0;
}

int quat_log(double q1[], double q2[], double log_q[])
//Calculates logarithm of orientation difference between quaternions
{
    double q[4], q2c[4], theta;
    int i;

    quat_conjugate(q2, q2c);
    quat_mult(q1, q2c, q);

    // Normalization to prevent any crashes
    double tmp;
    qnorm(q,tmp);
    for (i = 0; i < 4; i++)            q[i] = q[i] / tmp;

    tmp = 0;
    for (i = 1; i <= 3; i++)            tmp += q[i]*q[i];
    tmp = sqrt(tmp);

    if (tmp > eps){
        for (i = 0; i <= 2; i++)
            log_q[i] = acos(q[0]) * q[i+1] / tmp;
    }else
        log_q[0] = log_q[1] = log_q[2] = 0;

    return 0;
}

int normalize(double q[], double qnew[])
{
    double norm;
    qnorm(q,norm);
    int i;
    for (i=0;i<=3;i++)  qnew[i]=q[i]/norm;

    return 0;
}

int vnorm(double v[], double& vnorm)
{
    vnorm = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);

    return 0;
}


int qnorm(double q[], double& qnorm)
{
    qnorm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);

    return 0;
}

}

