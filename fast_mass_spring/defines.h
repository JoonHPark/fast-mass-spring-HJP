#pragma once
#include <Eigen/Sparse>

#define MIN_Hz 30.0
#define MAX_dT_nano 1.0 / MIN_Hz * 1000000.0

// horizontal (+x) node count 
#define ROW 2

// vertical (-z) node count
#define COL 2

// default spring length
#define REST_LENGTH 0.2

#define GRAVITY 100.0
#define MASS 1.0 / (ROW*COL)
#define STIFFNESS 10.0
#define STIFFNESS_CONSTRAINT 10.0 * STIFFNESS
#define DAMPING_NUMERICAL 0.0
#define DAMPING_PHYSICAL 0.0

// update
#define STEP_SIZE 1.0
#define MAX_ITER 10
#define EPSILON_ERROR 0.00001
#define EPSILON_PD 0.0005


// typedef
typedef Eigen::Matrix<double, 3, 3, 0, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 3, 1, 0, 3, 1> Vector3;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorX;
typedef Eigen::SparseMatrix<double> MatrixX;
