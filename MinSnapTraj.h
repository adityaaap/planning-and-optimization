#ifndef MINSNAPTRAJ_H
#define MINSNAPTRAJ_H

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/SparseLU>

#define EIGEN_RUNTIME_NO_MALLOC
#define EIGEN_NO_DEBUG


using namespace std;
using namespace Eigen;

class MinimumSnap{
public:
    MatrixXd globalPath;
    MatrixXd obstacles;
    double velocity;
    double dt;
    int n_coeffs;
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmin;
    double zmax;
    vector<double> time;
    MatrixXd A;
    MatrixXd B;
    int noSplines;
    vector<VectorXd> positions;
    vector<VectorXd> velocities;
    vector<VectorXd> acc;
    vector<int> spline_id;
    MatrixXd coeffs;
    int rowCounter;
    MatrixXd completeTraj;

    MinimumSnap(MatrixXd globalPath, MatrixXd obstacles, double velocity, double dt);

    void reset();
    MatrixXd getTrajectory();
    void generateTrajectroy(string method);
    void computeSplineParameters(string method);
    void createPolyMatrices();
    void setup();
    void positionConstraints();
    void startGoalConstraints();
    void continuityConstraints();
    void initMatrices();
    void numberSplines();
    void computeTime();
    void toCSV(vector<VectorXd> data);
    static VectorXd generatepolynomial(int noCoeffs, int order, double t);

};
#endif