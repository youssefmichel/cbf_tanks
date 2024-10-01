#pragma once
#include "eigen3/Eigen/Dense"
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <cfloat>
#include "vector"
#include <fstream>
#include "iostream"
using namespace std;
using namespace Eigen ;
#ifndef PASSIVE_DS_TYPEDEFS_H
#define PASSIVE_DS_TYPEDEFS_H


	//#ifdef USE_DOUBLE_PRECISION
	typedef double realtype;
	/*#else
	typedef float realtype;
	#endif*/
	typedef Eigen::Matrix<realtype, Eigen::Dynamic, Eigen::Dynamic> Mat;
	typedef Eigen::Matrix<realtype, Eigen::Dynamic, 1> Vec;

#endif
         const realtype pi=         3.141592653589793238462643383279502884 /* pi */;

         template <typename T> int sgn(T val) {
             return (T(0) < val) - (val < T(0));
         }

namespace special_math_functions //optional
{

	Mat Skew(const Vec &v);
        Quaterniond quatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2) ;
        Vec quatlog(Eigen::Quaterniond q1) ;
        Vec LogMapRot(Mat R)  ;
        Vec Rot_error(Mat R,Mat R_d) ;
        Vec quatlog_error(Mat R_a, Mat R_d) ;
        Mat Robust_inverse(Mat M) ;
        Mat SVD_sqrt(Mat M) ;
        Mat Eig_Decomp_Sqrt(Mat M) ;



}
namespace StiffnessProfiles
{
	Mat StiffnessProfile_MainTask(realtype t);
	Mat StiffnessProfile_NullSpace(realtype t);


}
namespace GeneralFunction {
	void Write_To_File(string file_name, vector<vector<float> > Input_Data);
	realtype smooth_transition_rising(realtype t, realtype t_max, realtype t_min, realtype scale);
	realtype smooth_transition_fall(realtype E, realtype E_max, realtype E_min);
    void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) ;
    realtype mod(realtype x, realtype y) ;
    void  Vec2double(Vec a,double q[]) ;
    Vec double2Vec(double q[]) ;
    Mat Vector2D_2_Mat(vector<vector<float>> const &v,int n_cols) ;
    template <size_t size_x, size_t size_y>

    Mat double2Mat(double(&T)[size_x][size_y])
    {
        int rows = size_x;
        int cols = size_y;
        Mat out = Mat::Zero(rows, cols);
        for (int i = 0; i<rows; i++)
        {
            for (int j = 0; j<cols; j++)
            {
                out(i, j) = T[i][j];
            }

        }
        return out;
    }


    template<typename T>
    std::vector<T> slice(std::vector<T> const &v, int m, int n)
    {
        auto first = v.cbegin() + m;
        auto last = v.cbegin() + n + 1;

        std::vector<T> vec(first, last);
        return vec;
    }

   template<typename T>
   vector<T> Get_Column(vector<vector<T>> const &v, int k)
    {
        vector<T> vec ;

       for (int i=0; i<v.size();i++){
          vec.push_back(v[i].at(k));
       }
       return vec ;


    }




}
