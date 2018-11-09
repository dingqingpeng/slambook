#include <iostream>
#include <ctime>
using namespace std;

// The Eigen part
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

/***************************************
 * Basic implementation of the Eigen lib
***************************************/

int main( int argc, char** argv )
{
    // Declare a 2x3 matrix
    Eigen::Matrix<float, 2, 3> matrix_23;
    
    // Vector3d <-- typedef Eigen::Matrix<double, 3, 1>
    Eigen::Vector3d v_3d;
    
    // Matrix3d <-- typedef Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    
    // Use dynamic matrix if size of matrix is not clear
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic;
    
    // More simply
    Eigen::MatrixXd matrix_x;
    
    
    // Input data
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // Output data
    cout << matrix_23 << endl;
    
    // Use () to read matrix elements
    for (int i = 0; i < 1; i++)
        for (int j = 0; j < 2; j++)
            cout << matrix_23(i, j) << endl;
    
    v_3d << 3, 2, 1;
    
    // Matrix times Vector
    // They must be the same type. The following computation is wrong
    // Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    
    // What should be done is to convert type explicitly
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;
    
    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;
    
    cout << matrix_33.transpose() << endl;
    cout << matrix_33.sum()       << endl;
    cout << matrix_33.trace()     << endl;
    cout << 10 * matrix_33        << endl;
    cout << matrix_33.inverse()   << endl;
    cout << matrix_33.determinant() << endl;
    
    Eigen::SelfAdjointEigenSolver< Eigen::Matrix3d > eigen_solver ( matrix_33.transpose() * matrix_33 );
    cout << "Eigen values = " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = " << eigen_solver.eigenvectors() << endl;
    
    // Solve equation matrix_NN * x = v_Nd
    Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE > matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    Eigen::Matrix< double, MATRIX_SIZE, 1 > v_Nd;
    v_Nd = Eigen::MatrixXd::Random( MATRIX_SIZE, 1 );
    
    clock_t time_stt = clock();
    
    // Solve using inverse
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time use in normal inverse is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    
    // Solve using QR decomposition
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time use in QR composition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    
    return 0;
}




