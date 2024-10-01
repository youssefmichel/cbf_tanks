
//
// Created by abudakka on 15/2/21.
//
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <ctime>
#include <vector>

#ifndef eps
#define eps 1.0e-12
#endif


#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

namespace  QuatUtils {


int quat_conjugate(double q1[], double q2[]);


int quat_mult(double q1[], double q2[], double q[])
// Quaternion multiplication
;

int quat_exp(double omega[], double q[])
//Quaternion exponential
;


int quat_log(double q1[], double q2[], double log_q[])
//Calculates logarithm of orientation difference between quaternions
;


int normalize(double q[], double qnew[])
;


int vnorm(double v[], double& vnorm)
;



int qnorm(double q[], double& qnorm) ;


}
