#include <iostream>
#include <Eigen/Geometry>
using namespace std;

int main(int argc, char const *argv[])
{
	// Object coordinate observed by robot 1
	Eigen::Vector4d p(0.5, 0, 0.2, 1);

	// Object coordinate under world coord
	Eigen::Vector4d p_w = Eigen::Vector4d::Zero();

	// Pose of robot 1
	// Initialize: ( w, x, y, z )
	Eigen::Quaterniond q1( 0.35, 0.2, 0.3, 0.1 );
	q1.normalize();
	// coeffs(): ( x, y, z, w )
	// cout << "q1 normalized = \n" << q1.coeffs() << endl;

	// Position of robot 1
	Eigen::Vector3d t2(0.3, 0.1, 0.1);

	// Make homogenous matrix
	// For Euler transform matrix, use Eigen::Isometry
	Eigen::Isometry3d T_cw_r1 = Eigen::Isometry3d::Identity();
	T_cw_r1.rotate( q1 );
	T_cw_r1.pretranslate(t2);
	// cout << "Transform matrix = \n" << T_cw.matrix() << endl;

	Eigen::Matrix< double, 4, 4 > t_cw_r1 = T_cw_r1.matrix();

	// Solve equation p = t_cw * p_w
	p_w = t_cw_r1.colPivHouseholderQr().solve(p);
	// cout << "p_w = " << p_w.transpose() << endl;

	// Pose and position of robot 2
	Eigen::Quaterniond q2( -0.5, 0.4, -0.1, 0.2 );
	q2.normalize();
	Eigen::Vector3d t(-0.1, 0.5, 0.3);
	// Make homogenous matrix
	Eigen::Isometry3d T_cw_r2 = Eigen::Isometry3d::Identity();
	T_cw_r2.rotate( q2 );
	T_cw_r2.pretranslate(t);

	Eigen::Vector4d p_r2 = T_cw_r2 * p_w;
	cout .precision(3);
	cout << "Object coord under robot 2 = " << p_r2.transpose().block<1, 3>(0, 0) << endl;
	
	return 0;
}