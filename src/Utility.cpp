
#include "Utility.h"

namespace special_math_functions {
	Mat Skew(const Vec &v) {
		if (v.size() == 1) {
			Mat out = Mat::Zero(2, 2);

			out << 0, -v(0),
				v(0), 0;
			return out;

		}
		else
		{
			Mat out = Mat::Zero(3, 3);
			out << 0, -v(2), v(1),
				v(2), 0, -v(0),
				-v(1), v(0), 0;
			return out;
		}


	}

    Vec quatlog_error(Mat R_a, Mat R_d){

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

   error = quatlog( quatMult(orientation_d,orientation.conjugate()) ) ;

   return error ;

}

    Eigen::Quaterniond quatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
        Eigen::Quaterniond resultQ;
        resultQ.setIdentity();

        resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
        resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

        return resultQ;
    }


    Vec quatlog(Eigen::Quaterniond q1){

        if(q1.vec().norm() > 0.00001){


            return acos( (float) q1.w() )  * (q1.vec()/q1.vec().norm()  ) ;

        }

       return  Vec::Zero(3) ;

        }




Vec LogMapRot(Mat R)  {


    Mat tmp =R-Mat::Identity(3,3) ;

    Vec r_log (3);

    r_log  <<0 ,0,0  ;



    if(tmp.norm()<0.00001){

     return r_log ;

    }

    else {
   realtype tmp=( R.trace() -1) /2 ;

    realtype theta=acos( (float) tmp ) ;

    Vec n(3) ;

    n   << R(2,1)-R(1,2) ,

           R(0,2)-R(2,0) ,

           R(1,0)-R(0,1)  ;


    n= 0.5*sin(theta)* n ;

    r_log=  theta*n ;

    return r_log  ;


    }


}


Vec Rot_error(Mat R,Mat R_d){

    return LogMapRot(R.transpose()*R_d) ;


}

Mat Robust_inverse(Mat M){

    MatrixXd M_f=M   ;
    JacobiSVD<MatrixXd> svd(M_f, ComputeThinU | ComputeThinV);
    realtype eps=0.00001 ;
    Vec sigma_rob=svd.singularValues() ;

    for (int i=0;i<sigma_rob.size();i++){
        if(sigma_rob(i) <eps){
            sigma_rob(i)=eps ;
        }

    }
    Mat Sigma_inverse=sigma_rob.asDiagonal() ;
    Mat M_inv= svd.matrixU() * Sigma_inverse.inverse() * svd.matrixV().transpose();
    return M_inv;

}

Mat SVD_sqrt(Mat M){

    MatrixXd M_f=M   ;

    JacobiSVD<MatrixXd> svd(M_f, ComputeThinU | ComputeThinV);
    realtype eps=0.00001 ;
    Vec sigma_rob=svd.singularValues() ;

    for (int i=0;i<sigma_rob.size();i++){

        sigma_rob(i)=sqrt(sigma_rob(i)) ;

    }
    Mat Sigma_sqrt=sigma_rob.asDiagonal() ;
    Mat M_inv= svd.matrixU() * Sigma_sqrt * svd.matrixV().transpose();
    return M_inv;

}

Mat Eig_Decomp_Sqrt(Mat M){

    EigenSolver<MatrixXd> ces;
    ces.compute(M);
    Mat V=  ces.eigenvectors().real()  ;
    Vec eig_val_sqrt= ces.eigenvalues().real() ;

    for (int i=0;i<eig_val_sqrt.size();i++){
        eig_val_sqrt(i)=sqrt(eig_val_sqrt(i)) ;
    }



    return V * eig_val_sqrt.asDiagonal() * V.inverse()  ;


}







}





namespace StiffnessProfiles {
	 

	Mat StiffnessProfile_MainTask(realtype t) {

		//temp 
		realtype k = 1*GeneralFunction::smooth_transition_rising(t, 4.0, 0, 600);

		return  Mat::Identity(3, 3)*k;


	}

	Mat StiffnessProfile_NullSpace(realtype t) {

		//	realtype k = 10* t;
        realtype k = 1 * GeneralFunction::smooth_transition_rising(t, 5.0, 0, 18); // Originally (t, 4.0, 0, 16)

        return  Mat::Identity(7, 7)*k;


	}



}
namespace GeneralFunction {


using namespace std;


	void Write_To_File(string file_name, vector<vector<float> > Input_Data) {
		ofstream myfile;
		myfile.open(file_name);

        if (!myfile)
        {
            cout << "No file found: " << file_name << endl;
            return;
        }

		for (int i = 0; i < Input_Data.size(); i++)
		{
			for (int j = 0; j < Input_Data[i].size(); j++)
			{

				if (j == Input_Data[i].size() - 1) {
					myfile << Input_Data[i][j] << endl;



				}
				else {

					myfile << Input_Data[i][j] << "  ";
				}


			}
		}
		myfile.close();
	}
	realtype smooth_transition_rising(realtype t, realtype t_max, realtype t_min, realtype scale){

	realtype alpha_min = 0.0;
	realtype alpha;

	if (t>t_max)
		alpha = 0;
	else if(t<t_min)
		alpha = 1;
	else
		alpha = (0.5*(1 + cos(pi / (t_max - t_min)*(t - t_min)))*(1 - alpha_min) + alpha_min);
	
	
     alpha = 1 - alpha;

    return  scale*alpha;

	}

realtype smooth_transition_fall(realtype E, realtype E_max, realtype E_min){
	realtype alpha_min = 0.0;
	realtype alpha;

	if (E>E_max)
		alpha = 0;
	else if(E<E_min)
		alpha = 1;
	else
		alpha = (0.5*(1 + cos(pi / (E_max - E_min)*(E - E_min)))*(1 - alpha_min) + alpha_min);
	

		return  1 - alpha;
	}
	
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

realtype mod(realtype x, realtype y){
    return x - floor(x/y)*y ;
}

void  Vec2double(Vec a,double q[]) {

    int s = a.size();
    for (int i = 0; i < s; i++) {

        q[i] = a(i);
    }

}
Vec double2Vec(double q[]) {
    int s = (sizeof(q) / sizeof(*q));
    Vec out = Vec::Zero(s);
    for (int i = 0; i < s; i++) {

        out[i] = q[i];
    }
    return out;

}



Mat Vector2D_2_Mat(vector<vector<float>> const &v,int n_cols){

    Mat out=Mat::Zero(v.size(),n_cols) ;

    for (int i=0; i<v.size();i++){
     for (int k=0; k< n_cols ; k++){
       out(i,k)=v.at(i).at(k);
    }
    }
    return out  ;


}
}
