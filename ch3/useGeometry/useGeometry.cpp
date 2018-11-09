#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

/**************************
 * Demonstration of Eigen Geometry module
 *************************/

int main(int argc, char const *argv[])
{
	// Use Matrix3d / Matrix3f as 3D rotation matrix
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	// Use AngleAxis as rotation vector
	// Rotate 45 degrees along axis z
	Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d( 0, 0, 1 ) );

	cout .precision(3);
	cout << "rotation matrix =\n" << rotation_vector.matrix() << endl; // Convert to matrix using matrix()

	// Or assign directly
	rotation_matrix = rotation_vector.toRotationMatrix();

	// Use AngleAxis to implement transform
	Eigen::Vector3d v ( 1, 0, 0 );
	Eigen::Vector3d v_rotated = rotation_vector * v;
	cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;

	// Or use rotation matrix
	v_rotated = rotation_matrix * v;
	cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;

	// Euler Angles
	Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2, 1, 0 );  // ZXY, AKA yaw pitch roll
	cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

	// Use Eigen::Isometry as transform matrix
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  	// It's a 4x4 matrix actually
	T.rotate ( rotation_vector );							// Rotate as rotation_matrix
	T.pretranslate ( Eigen::Vector3d ( 1, 3, 4 ) );			// Translate vector set to ( 1, 3, 4 )
	cout << "Transfor matrix = \n" << T.matrix() << endl;

	// Use transform matrix to implement transform
	Eigen::Vector3d v_transformed = T * v;
	cout << "v transformed = " << v_transformed.transpose() << endl;

	// Quaternion
	// It is ok to directly assign AngleAxis to quaternion, and vise versa
	Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
	cout << "quaternion = \n" << q.coeffs() << endl; // coeffs ==> (x, y, z, w), where w is the real part

	// Rotation matrix can be assigned to quaternoid
	q = Eigen::Quaterniond ( rotation_matrix );
	cout << "quaternion = \n" << q.coeffs() << endl;

	// use quaternoid to rotate a vector
	v_rotated = q * v;
	cout << "(1, 0, 0) after rotation" << v_rotated.transpose() << endl;

	return 0;
}