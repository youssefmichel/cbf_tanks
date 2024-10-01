#include "ft_comp.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;

int main() {


   /* Eigen::Matrix<double, 3,3> R;
    R.setIdentity();

    //cout << R << endl;

    double com[3] = {0, 0, 0.1};
    double bias[6] = {0,0,0,0,0,0};
    double ft_measured[]={1,2,3,4,5,6};*/

    double com[3] = {0, 0, 0.1};

    ft_comp ft(1.0, com); //Objekt ft wird erzeugt

    // robot_pose 12 vector
    // R:
    // 0,1,2
    // 4,5,6
    // 8,9,10
    // P (x,y,z): 3,7,11

    //double robot_pose[12];
    //double comp_values[6];

    ft.compensate();
    ft.final();


    /*ft.compensate(robot_pose, comp_values);
    ft.reset();*/


}
