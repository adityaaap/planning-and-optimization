#include <vector>
#include <array>
#include <iostream>
#include "RRTStar.h" 
#include "MinSnapTraj.h" 
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {
    vector<double> start = {0,0,0};
    vector<double> goal = {70,70,70}; // 7.0*10 for each coordinate
    std::vector<std::vector<double>> space_limits = {{0., 0., 0.9}, {100., 100., 100.}};
    std::vector<std::vector<double>> obs = {{15, 55, 15, 55, 15, 55}};

    RRTStar rrt(
        space_limits,
        start,
        goal,
        5, // max_distance
        1000, // max_iterations
        obs
    );

    rrt.run();
    std::vector<std::vector<double>> global_path = rrt.bestPath;
    MatrixXd matrix_path(global_path.size(), global_path[0].size());

    for (int i = 0; i < global_path.size(); ++i) {
        for (int j = 0; j < global_path[0].size(); ++j) {
            matrix_path(i, j) = global_path[i][j];
        }
    }

    MatrixXd obs_optim(1, 6);
    obs_optim << 15,55,15,55,15,55; //xmin, xmax, ymin, ...

    std::cout << "start traj optim" << std::endl;
    MinimumSnap min_snap(matrix_path, obs_optim, 2, 0.1);

    MatrixXd global_trajectory = min_snap.getTrajectory();
    vector<VectorXd> positions = min_snap.positions;
    for (int i = 0; i < positions.size(); i++) {
        cout << "Position " << i << ": " << positions[i].transpose() << endl;
    }
    min_snap.toCSV(positions);

    return 0;
}


