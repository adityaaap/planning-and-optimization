#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/SparseLU>
#include "MinSnapTraj.h"

#define EIGEN_RUNTIME_NO_MALLOC
#define EIGEN_NO_DEBUG


using namespace std;
using namespace Eigen;


MinimumSnap::MinimumSnap(MatrixXd globalPath, MatrixXd obstacles, double velocity, double dt) {
        this->globalPath = globalPath;
        this->obstacles = obstacles;
        this->velocity = velocity;
        this->dt = dt;
        this->n_coeffs = 8;
        this->xmin = this->obstacles(0, 0);
        this->xmax = this->obstacles(0, 1);
        this->ymin = this->obstacles(0, 2);
        this->ymax = this->obstacles(0, 3);
        this->zmin = this->obstacles(0, 4);
        this->zmax = this->obstacles(0, 5);
        this->time.clear();
        this->A.resize(0, 0);
        this->B.resize(0, 0);
        this->noSplines = 0;
        this->positions.clear();
        this->velocities.clear();
        this->acc.clear();
        this->spline_id.clear();
        this->coeffs.resize(0, 0);
        this->rowCounter = 0;
        this->completeTraj.resize(0, 0);
}
    
    void MinimumSnap::reset() {
        this->time.clear();
        this->A.resize(0, 0);
        this->B.resize(0, 0);
        this->noSplines = 0;
        this->positions.clear();
        this->velocities.clear();
        this->acc.clear();
        this->spline_id.clear();
        this->coeffs.resize(0,0);
        this->rowCounter = 0;
        this->completeTraj.resize(0, 0);
}
    
MatrixXd MinimumSnap::getTrajectory() {
        string method = "lstq";
        this->generateTrajectroy(method);
        return this->completeTraj;
}
    
void MinimumSnap::generateTrajectroy(string method) {
        this->computeSplineParameters(method);
        
        for (int i = 0; i < this->noSplines; i++) {
            double time = this->time[i];
            for (double tT = 0.0; tT < time; tT += this->dt) {
                VectorXd pos = this->generatepolynomial(this->n_coeffs, 0, tT).transpose() * this->coeffs.block(i * this->n_coeffs, 0, this->n_coeffs, this->coeffs.cols());
                VectorXd vel = this->generatepolynomial(this->n_coeffs, 1, tT).transpose() * this->coeffs.block(i * this->n_coeffs, 0, this->n_coeffs, this->coeffs.cols());
                VectorXd acc = this->generatepolynomial(this->n_coeffs, 2, tT).transpose() * this->coeffs.block(i * this->n_coeffs, 0, this->n_coeffs, this->coeffs.cols());

                // VectorXd pos = this->generatepolynomial(this->n_coeffs, 0, tT).transpose() * this->coeffs.segment(i * this->n_coeffs, this->n_coeffs);
                // VectorXd vel = this->generatepolynomial(this->n_coeffs, 1, tT).transpose() * this->coeffs.segment(i * this->n_coeffs, this->n_coeffs);
                // VectorXd acc = this->generatepolynomial(this->n_coeffs, 2, tT).transpose() * this->coeffs.segment(i * this->n_coeffs, this->n_coeffs);
                this->positions.push_back(pos);
                this->velocities.push_back(vel);
                this->acc.push_back(acc);
            }
        }
        
        int numCols = this->positions[0].size();
        this->completeTraj.resize(this->positions.size(), 3 * numCols);
        for (int i = 0; i < this->positions.size(); i++) {
            this->completeTraj.block(i, 0, 1, numCols) = this->positions[i].transpose();
            this->completeTraj.block(i, numCols, 1, numCols) = this->velocities[i].transpose();
            this->completeTraj.block(i, 2 * numCols, 1, numCols) = this->acc[i].transpose();
        }
}
    
void MinimumSnap::computeSplineParameters(string method) {
        this->createPolyMatrices();
        if (method == "lstq") {
            //this->coeffs.resize(this->A.rows(), this->B.cols());
            //this->coeffs = this->A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(this->B);
            Eigen::MatrixXd pseudoInverse = (this->A.transpose() * this->A).ldlt().solve(this->A.transpose());
            this->coeffs = pseudoInverse * this->B;
        } else {
            this->coeffs = this->A.fullPivLu().solve(this->B);
        }
}
    
void MinimumSnap::createPolyMatrices() {
        this->setup();
        this->positionConstraints();
        this->startGoalConstraints();
        this->continuityConstraints();
}
    
void MinimumSnap::setup() {
        this->numberSplines();
        this->computeTime();
        this->initMatrices();
}
    
void MinimumSnap::positionConstraints() {
        for (int i = 0; i < this->noSplines; i++) {
            VectorXd poly = this->generatepolynomial(this->n_coeffs, 0, 0);
            this->A.row(this->rowCounter).segment(i * this->n_coeffs, this->n_coeffs) = poly;
            this->B.row(this->rowCounter) = this->globalPath.row(i);
            this->rowCounter++;
        }
        
        for (int i = 0; i < this->noSplines; i++) {
            double tT = this->time[i];
            VectorXd poly = this->generatepolynomial(this->n_coeffs, 0, tT);
            this->A.row(this->rowCounter).segment(i * this->n_coeffs, this->n_coeffs) = poly;
            this->B.row(this->rowCounter) = this->globalPath.row(i + 1);
            this->rowCounter++;
        }
}
    
void MinimumSnap::startGoalConstraints() {
        for (int k = 1; k <= 3; k++) {
            VectorXd poly = this->generatepolynomial(this->n_coeffs, k, 0);
            this->A.row(this->rowCounter).segment(0, this->n_coeffs) = poly;
            this->rowCounter++;
        }
        
        for (int k = 1; k <= 3; k++) {
            VectorXd poly = this->generatepolynomial(this->n_coeffs, k, this->time[this->noSplines - 1]);
            this->A.row(this->rowCounter).segment((this->noSplines - 1) * this->n_coeffs, this->n_coeffs) = poly;
            this->rowCounter++;
        }
}
    
void MinimumSnap::continuityConstraints() {
        for (int i = 1; i < this->noSplines; i++) {
            double timeT = this->time[i - 1];
            for (int k = 1; k <= 4; k++) {
                VectorXd poly0 = -1 * this->generatepolynomial(this->n_coeffs, k, 0);
                VectorXd polyT = this->generatepolynomial(this->n_coeffs, k, timeT);
                VectorXd poly(2 * this->n_coeffs);
                poly << polyT, poly0;
                
                this->A.row(this->rowCounter).segment((i - 1) * this->n_coeffs, 2 * this->n_coeffs) = poly;
                this->rowCounter++;
            }
        }
}
    
void MinimumSnap::initMatrices() {
        int numRows = this->n_coeffs * this->noSplines;
        int numCols = this->globalPath.cols();
        this->A.resize(numRows, numRows);
        this->B.resize(numRows, numCols);
}
    
void MinimumSnap::numberSplines() {
        this->noSplines = this->globalPath.rows() - 1;
}
    
void MinimumSnap::computeTime() {
        double vel = this->velocity;
        for (int i = 0; i < this->noSplines; i++) {
            double dist = (this->globalPath.row(i + 1) - this->globalPath.row(i)).norm();
            double time1 = dist / this->velocity;
            this->time.push_back(time1);
        }
}

void MinimumSnap::toCSV(vector<VectorXd> data){
        string name = "output_traj_optim.csv";
        ofstream csvFile(name);

        // if(!csvFile.is_open()){
        //     return "error";
        // }

        for(int i=0; i<data.size(); i++){
            for(int j=0; j<data[i].size(); j++){
                csvFile << data[i][j];
                if(j < data.size()-1){
                    csvFile << ",";
                }
            }
            csvFile << "\n";
        }

        csvFile.close();
        cout<< "Data has been writen"<< endl;
}
    
    
VectorXd MinimumSnap::generatepolynomial(int noCoeffs, int order, double t) {
        VectorXd poly(noCoeffs);
        VectorXd deri(noCoeffs);
        for (int i = 0; i < noCoeffs; i++) {
            poly(i) = 1;
            deri(i) = i;
        }
        
        for (int _ = 0; _ < order; _++) {
            for (int j = 0; j < noCoeffs; j++) {
                poly(j) = poly(j) * deri(j);
                if (deri(j) > 0) {
                    deri(j) = deri(j) - 1;
                }
            }
        }
        
        for (int i = 0; i < noCoeffs; i++) {
            poly(i) = poly(i) * pow(t, deri(i));
        }
        
        return poly;
}


// Uncomment for Unit Testing of Code


// int main() {
//     cout<< "Start Running" << endl;
//     MatrixXd global_path(4, 3);
//     global_path << 0.9, 1.9, 0,
//                    2, 0.75, 0,
//                    3.25, 2, 0,
//                    10, 10, 0;
    
//     MatrixXd obs(1, 6);
//     obs << 1, 3, 1, 3, 0, 0;
//     double xmin = 1;
//     double xmax = 3;
//     double ymin = 1;
//     double ymax = 3;
//     double zmin = 0;
//     double zmax = 0;
    
//     MinimumSnap min_snap(global_path, obs, 2, 0.1);
//     cout<< "Start Running2" << endl;
    
//     MatrixXd global_trajectory = min_snap.getTrajectory();
//     vector<VectorXd> positions = min_snap.positions;

//     min_snap.toCSV(positions);

//     for (int i = 0; i < positions.size(); i++) {
//         cout << "Position " << i << ": " << positions[i].transpose() << endl;
//     }
    
//     //min_snap.plot();
    
//     return 0;
// }
