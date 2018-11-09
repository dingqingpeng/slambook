#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main(int argc, char const *argv[])
{
	// Rotate 90 degrees along Z axis
	Eigen::Matrix3d R = Eigen::AngleAxisd( M_PI/2, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
	
	Sophus::SO3 SO3_R( R );				// construct using a rotation matrix
	Sophus::SO3 SO3_v( 0, 0, M_PI/2 );	// construct using a rotation vector,
										// which is actually a kind of Lie algebra
	Eigen::Quaterniond q( R );			// Or construct using quaternoin
	Sophus::SO3 SO3_q( q );

	cout << "SO(3) from matrix: " << SO3_R << endl;
	cout << "SO(3) from vector: " << SO3_v << endl;
	cout << "SO(3) from quaternoin: " << SO3_q << endl;

	Eigen::Vector3d so3 = SO3_R.log();
	cout << "so3 = " << so3.transpose() << endl;

	cout << "so3 hat = \n" << Sophus::SO3::hat(so3) << endl;
	cout << "so3 hat vee = " << Sophus::SO3::vee( Sophus::SO3::hat(so3) ).transpose() << endl;

	// Update disturbance model
	Eigen::Vector3d update_so3(1e-4, 0, 0);
	Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
	cout << "SO3 updated = " << SO3_updated << endl;

	/*
	 * Operation on SE(3)
	 */
	Eigen::Vector3d t(1, 0, 0);		// Translate along axis X
	Sophus::SE3 SE3_Rt(R, t);		// Construct SE(3) from R & t
	Sophus::SE3 SE3_qt(q, t);		// Construct SE(3) from q & t
	cout << "SE3 from R, t = \n" << SE3_Rt << endl;
	cout << "SE3 from q, t = \n" << SE3_qt << endl;

	// se(3) is a 6-degree vector
	typedef Eigen::Matrix< double, 6, 1 > Vector6d;
	Vector6d se3 = SE3_Rt.log();
	cout << "se3 = " << se3.transpose() << endl;

	cout << "se3 hat = \n" << Sophus::SE3::hat(se3) << endl;
	cout << "se3 hat vee = \n" << Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose() << endl;

	Vector6d update_se3;
	update_se3.setZero();
	update_se3(0, 0) = 1e-4;
	Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
	cout << "SE3 updated = \n" << SE3_updated.matrix() << endl;

	return 0;
}