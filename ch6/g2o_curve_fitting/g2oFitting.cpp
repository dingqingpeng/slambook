#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <opencv2/core/core.hpp>
#include <chrono>

using namespace std;

/* Vertex model
 * Template parameter:
 *     dimension of optimized variable
 *     type of optimized variable
 */
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl()  // Reset
    {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl( const double* update )  // Update
    {
        _estimate += Eigen::Vector3d(update);
    }
    // Read and Write: Leave empty
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
};

/* Edge model
 * Template parameter:
 *     dimension of observed variable
 *     type of observed variable
 *     type of connected vertex
 */
class CurveFittingEdge: public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    // Compute curve model error
    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp( abc(0, 0)*_x*_x + abc(1, 0)*_x + abc(2, 0) );
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x;
};

int main(int argc, char const *argv[])
{
    //! [parameters to generate real data points]
    double a = 1.0, b = 2.0, c = 1.0;
    int N = 100;
    double w_sigma = 1.0;
    cv::RNG rng;
    vector<double> x_data, y_data;
    //! End of [parameters to generate real data points]

    //! [estimated parameters]
    double abc[3] = {0, 0, 0};
    //! End of [estimated parameters]

    //! [generate data]
    cout << "Generating data: " << endl;
    for(int i = 0; i < N; i++)
    {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back( exp(a*x*x + b*x + c) + rng.gaussian(w_sigma) );
        // cout .precision(3);
        // cout << x_data[i] << ", " << y_data[i] << endl;
    }
    //! End of [generate data]

    //! [construct graph optimization]
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 1> > Block; // Optimized variable dimension is 3, observed variable / error dimension is 1
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();    // Linear solver
    Block* solver_ptr = new Block( linearSolver );
    // Gradient decent method, choose from GaussNewton, Levenberg-Marquardt and DogLeg
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer;     // Graph model
    optimizer.setAlgorithm( solver );   // Set algorithm
    optimizer.setVerbose( true );       // Output debug information

    // Add vertex to graph
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0, 0, 0) );
    v->setId(0);
    optimizer.addVertex( v );

    // Add edges to graph
    for(int i = 0; i < N; i++)
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
        edge->setId(i);
        edge->setVertex(0, v);              // Set connected vertex
        edge->setMeasurement( y_data[i] );  // Observed value
        edge->setInformation( Eigen::Matrix<double, 1, 1>::Identity()*1/(w_sigma*w_sigma) );    // Information matrix: reverse of covariance matrix
        optimizer.addEdge( edge );
    }
    //! End of [construct graph optimization]

    //! [execute optimization]
    cout << "Start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout << "Solve time cost = " << time_used.count() << " seconds." << endl;
    //! End of [execute optimization]
    
    //! [output result]
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;
    //! End of [output result]
    
    return 0;
}
