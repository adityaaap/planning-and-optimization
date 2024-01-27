#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <map>
#include <random>
#include <algorithm>
#include <fstream>
#include <string>
#include <stdexcept>

class RRTStar {
public:
    std::vector<double> start;
    std::vector<double> goal;
    std::vector<double> goalError;
    int maxIterations;
    std::vector<std::vector<double>> obstacles;
    double maxDistance;
    std::vector<double> lower;
    std::vector<double> upper;
    double stepSize;
    double neighbourRadius;
    double epsilon;
    std::vector<double> currSample;
    int dynamic_it_counter;
    int dynamic_break_at;
    std::vector<std::vector<double>> spaceLimits;

    std::vector<std::vector<double>> allNodes;
    std::vector<std::vector<double>> bestPath;
    std::map<std::string, std::vector<double>> bestTree;
    std::map<std::string, std::vector<double>> tree;

    RRTStar(std::vector<std::vector<double>> spaceLimits, std::vector<double> start, std::vector<double> goal, double max_distance, int maxIterations, std::vector<std::vector<double>> obstacle);

    void updateTree(std::vector<double> node, std::vector<double> newNode);
    std::vector<double> generateNode();
    std::vector<double> findNearest(std::vector<double> newNode);
    std::vector<double> unitVectorToNode(std::vector<double> newNode, std::vector<double> nearestNode);
    std::vector<std::vector<double>> validNeighbours(std::vector<double> newNode);
    bool validConnection(std::vector<double> node, std::vector<double> newNode);
    std::vector<double> bestNeighbour(std::vector<std::vector<double>> neighbours);
    bool rewire(std::vector<std::vector<double>> neighbours, std::vector<double> newNode);
    bool isPathFound(std::map<std::string, std::vector<double>> tree, std::vector<double> newNode);
    std::vector<std::vector<double>> getPath(std::map<std::string, std::vector<double>> tree);
    void store_best_tree();
    static double path_cost(std::vector<std::vector<double>> path);
    void run();
    void writeDataToCSV(const std::vector<std::vector<double>>& data, const std::string& filename);
};

#endif // RRTSTAR_H
