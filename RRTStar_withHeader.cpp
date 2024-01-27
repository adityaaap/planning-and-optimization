#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <map>
#include <random>
#include <algorithm>
#include <fstream>
#include "RRTStar.h"

using namespace std;


RRTStar::RRTStar(vector<vector<double>> spaceLimits, vector<double> start, vector<double> goal, double max_distance, int maxIterations, vector<vector<double>> obstacle){
        this->start = start;
        this->goal = goal;
        this->goalError = {goal[0] - 0.05, goal[1] - 0.05, goal[2]-0.05};
        this->maxIterations = maxIterations;
        this->maxDistance = max_distance;
        this->obstacles = obstacle;
        this->lower = spaceLimits[0];
        this->upper = spaceLimits[1];
        this->allNodes.push_back(this->start);
        // allNodes[0] = this->start;

        // this->tree =

        this->stepSize = max_distance;
        this->neighbourRadius = 1.1 * this->maxDistance;
        this->epsilon = 0.3;
        this->currSample = {};

        this->bestPath = {};
        this->bestTree = {};

        this->dynamic_it_counter = 0;
        this->dynamic_break_at = this->maxIterations / 10;

}

void RRTStar::updateTree(vector<double> node, vector<double> newNode){
        this->allNodes.push_back(newNode);
        std::string key = to_string(newNode[0]) + "," + to_string(newNode[1]) + "," + to_string(newNode[2]);
        std::vector<double> parent = node;
        if(newNode != node){
            this->tree[key] = parent;
        }
}

std::vector<double> RRTStar::generateNode(){
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<double> dis(0.0, 1.0);
        if(dis(gen) < this->epsilon){
            return this->goal;
        }

        double x = (dis(gen) * (this->upper[0] - this->lower[0])) + this->lower[0];
        double y = (dis(gen) * (this->upper[1] - this->lower[1])) + this->lower[1];
        double z = (dis(gen) * (this->upper[2] - this->lower[2])) + this->lower[2];

        std::vector<double> randomNode = {std::round(x*100)/100, round(y*100)/100, round(z*100)/100};
        return randomNode;

        // ALWAYS GENERATES SAME NODES - IN OBSERVATION

        // if (std::rand() / (RAND_MAX + 1.0) < this->epsilon) {
        //     return this->goal;
        // }
        // double x = (std::rand() / (RAND_MAX + 1.0)) * (this->upper[0] - this->lower[0]) + this->lower[0];
        // double y = (std::rand() / (RAND_MAX + 1.0)) * (this->upper[1] - this->lower[1]) + this->lower[1];
        // double z = (std::rand() / (RAND_MAX + 1.0)) * (this->upper[2] - this->lower[2]) + this->lower[2];
        // std::vector<double> randomNode = {std::round(x*100)/100, std::round(y*100)/100, std::round(z*100)/100};
        // return randomNode;
}

std::vector<double> RRTStar::findNearest(vector<double> newNode){
        std::vector<double> distances;
        for(const auto& node : allNodes){
            distances.push_back(sqrt(pow(newNode[0] - node[0],2) + pow(newNode[1] - node[1],2) + pow(newNode[2] - node[2],2)));
        }

        std::vector<double> nearestNode = allNodes[distance(distances.begin(), min_element(distances.begin(), distances.end()))];
        return nearestNode;

        // std::vector<double> nearest_node;
        // double min_distance = std::numeric_limits<double>::max();
        // for (std::vector<double> node : this->allNodes) {
        //     double distance = std::sqrt(std::pow(newNode[0] - node[0], 2) + std::pow(newNode[1] - node[1], 2) + std::pow(newNode[2] - node[2], 2));
        //     if (distance < min_distance) {
        //         min_distance = distance;
        //         nearest_node = node;
        //     }
        // }
        // return nearest_node;
}

std::vector<double> RRTStar::unitVectorToNode(vector<double> newNode, vector<double> nearestNode){
        // vector<double> newNodeUnit;
        // // double nearest_distance = sqrt(pow(newNode[0]-nearestNode[0],2) + pow(newNode[1]-nearestNode[1],2) + pow(newNode[2]-nearestNode[2],2));
        // double distance_nearest = 0;
        // for (int i = 0; i < newNode.size(); i++) {
        //     distance_nearest += pow(newNode[i] - nearestNode[i], 2);
        // }
        // distance_nearest = sqrt(distance_nearest);
        
        // if(distance_nearest > this->stepSize){
        //     for(int i=0; i<newNode.size(); i++){
        //         newNodeUnit.push_back(nearestNode[i] + (newNode[i] - nearestNode[i]) * this->stepSize / distance_nearest);
        //     }
        //     for(int i=0; i<newNodeUnit.size(); i++){
        //         newNodeUnit[i] = round(newNodeUnit[i]*100)/100;
        //     }
        //     // newNodeUnit = nearestNode + (newNode - nearestNode) * this->stepSize / distance_nearest;
        // }
        // return newNodeUnit;

        // BETTER IN TIME COMPLEXITY
        double distance_nearest = std::sqrt(std::pow(newNode[0] - nearestNode[0], 2) + std::pow(newNode[1] - nearestNode[1], 2) + std::pow(newNode[2] - nearestNode[2], 2));
        if (distance_nearest > this->stepSize) {
            double x = nearestNode[0] + (newNode[0] - nearestNode[0]) * this->stepSize / distance_nearest;
            double y = nearestNode[1] + (newNode[1] - nearestNode[1]) * this->stepSize / distance_nearest;
            double z = nearestNode[2] + (newNode[2] - nearestNode[2]) * this->stepSize / distance_nearest;
            std::vector<double> new_node = {std::round(x*100)/100, std::round(y*100)/100, std::round(z*100)/100};
            newNode = new_node;
        }
        return newNode;
}

std::vector<std::vector<double>> RRTStar::validNeighbours(vector<double> newNode){
        // vector<vector<double>> neighbours;
        // double dist;
        // for(const auto& node : this->allNodes){
        //     for (int i = 0; i < newNode.size(); i++) {
        //         dist += pow(newNode[i] - node[i], 2);
        //     }
        //     dist = sqrt(dist);
        //     if(dist < this->neighbourRadius){
        //         bool flag = validConnection(node, newNode);
        //         if(flag){
        //             neighbours.push_back(node);
        //         }
        //     }
        // }

        // return neighbours;
        std::vector<std::vector<double>> neighbours;
        for (std::vector<double> node : this->allNodes) {
            double distance = std::sqrt(std::pow(node[0] - newNode[0], 2) + std::pow(node[1] - newNode[1], 2) + std::pow(node[2] - newNode[2], 2));
            if (distance <= this->neighbourRadius) {
                if (this->validConnection(node, newNode)) {
                    neighbours.push_back(node);
                }
            }
        }
        return neighbours;
}

bool RRTStar::validConnection(std::vector<double> node, std::vector<double> newNode){
        if (this->obstacles.empty()) {
            return true;
        }
        for (std::vector<double> obs : this->obstacles) {
            double xmin = obs[0];
            double xmax = obs[1];
            double ymin = obs[2];
            double ymax = obs[3];
            double zmin = obs[4];
            double zmax = obs[5];
            std::vector<double> node1 = node;
            std::vector<double> node2 = newNode;
            std::vector<double> direction = {node2[0] - node1[0], node2[1] - node1[1], node2[2] - node1[2]};
            std::vector<double> t(100);
            std::vector<std::vector<double>> points(100, std::vector<double>(3));
            for (int i = 0; i < 100; i++) {
                t[i] = i / 99.0;
                points[i][0] = node1[0] + t[i] * direction[0];
                points[i][1] = node1[1] + t[i] * direction[1];
                points[i][2] = node1[2] + t[i] * direction[2];
            }
            for (std::vector<double> point : points) {
                if (point[0] >= xmin && point[0] <= xmax && point[1] >= ymin && point[1] <= ymax && point[2] >= zmin && point[2] <= zmax) {
                    return false;
                }
            }
        }
        return true;
}

std::vector<double> RRTStar::bestNeighbour(std::vector<std::vector<double>> neighbours){
        // vector<double> cost;
        // for(vector<double> neigh : neighbours){
        //     double cst = sqrt(pow(neigh[0]-this->start[0],2) + pow(neigh[1]-this->start[1],2) + pow(neigh[2]-this->start[2],2));
        //     cost.push_back(cst);
        // }

        // vector<double> bestNeighbour = neighbours[distance(cost.begin(), min_element(cost.begin(), cost.end()))];

        // return bestNeighbour;

        double min_cost = std::numeric_limits<double>::max();
        std::vector<double> best_neighbour;
        for (std::vector<double> neighbour : neighbours) {
            double cost = std::sqrt(std::pow(neighbour[0] - this->start[0], 2) + std::pow(neighbour[1] - this->start[1], 2) + std::pow(neighbour[2] - this->start[2], 2));
            if (cost < min_cost) {
                min_cost = cost;
                best_neighbour = neighbour;
            }
        }
        return best_neighbour;
}

bool RRTStar::rewire(std::vector<std::vector<double>> neighbours, std::vector<double> newNode){
        for(std::vector<double> neigh : neighbours){
            if(neigh ==  this->tree[to_string(round(newNode[0]*100)/100) + "," + to_string(round(newNode[1]*100)/100) + "," + to_string(round(newNode[2]*100)/100)]){
                continue;
            }

            if(this->validConnection(neigh, newNode)){
                std::vector<double> currentParent = this->tree[to_string(round(neigh[0]*100)/100) + "," + to_string(round(neigh[1]*100)/100) + "," + to_string(round(neigh[2]*100)/100)];

                double currentCost = sqrt(pow(neigh[0]-this->start[0],2) + pow(neigh[1]-this->start[1],2) + pow(neigh[2]-this->start[2],2));
                
                double interCost = sqrt(pow(newNode[0]-this->start[0],2) + pow(newNode[1]-this->start[1],2) + pow(newNode[2]-this->start[2],2));
                
                double newCost = sqrt(pow(neigh[0]-newNode[0],2) + pow(neigh[1]-newNode[1],2) + pow(neigh[2]-newNode[2],2)) + interCost;

                if(newCost < currentCost){
                    this->tree[to_string(round(neigh[0]*100)/100) + "," + to_string(round(neigh[1]*100)/100) + "," + to_string(round(neigh[2]*100)/100)] = newNode;
                    return true;
                }
            }
        }

        return false;
}

bool RRTStar::isPathFound(map<string, vector<double>> tree, vector<double> newNode){
        std::string goal_node_key = std::to_string(std::round(this->goal[0]*100)/100) + "," + std::to_string(std::round(this->goal[1]*100)/100) + "," + std::to_string(std::round(this->goal[2]*100)/100);
        return tree.find(goal_node_key) != tree.end();
        
}

std::vector<std::vector<double>> RRTStar::getPath(map<string, vector<double>> tree){
        std::vector<std::vector<double>> path;
        std::vector<double> node = this->goal;
        while (node != this->start) {
            path.push_back(node);
            node = tree[std::to_string(std::round(node[0]*100)/100) + "," + std::to_string(std::round(node[1]*100)/100) + "," + std::to_string(std::round(node[2]*100)/100)];
        }
        std::reverse(path.begin(), path.end());
        return path;
        /*
        double cost = this->path_cost(path);
        pair<vector<vector<double>>, double> result;
        result.first = path;
        result.second = cost;
        */
        //return path;
}

void RRTStar::store_best_tree() {
        this->bestTree = this->tree;
}

double RRTStar::path_cost(vector<vector<double>> path){
        double cost = 0;
        for(int i=0; i<path.size()-1; i++){
            vector<double> node1 = path[i+1];
            vector<double> node2 = path[i];
            cost += sqrt(pow(node1[0]-node2[0],2) + pow(node1[1]-node2[1],2) + pow(node1[2]-node2[2],2));
        }
        return cost;
}

void RRTStar::run(){
        double old_cost = std::numeric_limits<double>::infinity();
        for(int i=0; i<this->maxIterations; i++){
            std::vector<double> node = this->generateNode();
            std::vector<double> neartestNode = this->findNearest(node);
             this->currSample = this->unitVectorToNode(node, neartestNode);
            std::vector<std::vector<double>> neighbours = this->validNeighbours(this->currSample);
            if(neighbours.empty()){
                continue;
            }

            std::vector<double> best_Neighbour = bestNeighbour(neighbours);
            this->updateTree(best_Neighbour, this->currSample);
            bool hasRewired = this->rewire(neighbours, this->currSample);
            if(this->isPathFound(this->tree, this->currSample)){
                cout<<"here in run if path found"<<endl;
                std::vector<std::vector<double>> path = this->getPath(this->tree);
                //double cost = result.second;
                this->store_best_tree();
                std::cout << "iterations:"<< i <<std::endl;
                break;
            }
        }

        if (!this->isPathFound(this->bestTree, this->currSample)) {
            throw std::runtime_error("No path found");
        }
        this->bestPath = this->getPath(this->bestTree);
        //this->bestPath = result1.first;
        std::cout << "Best path found with cost: " << RRTStar::path_cost(this->bestPath) << std::endl;
} 

void RRTStar::writeDataToCSV(const std::vector<std::vector<double>>& data, const std::string& filename) {
        std::ofstream outputFile(filename);

        if (outputFile.is_open()) {
            for (const auto& row : data) {
                for (size_t i = 0; i < row.size(); ++i) {
                    outputFile << row[i];
                    if (i < row.size() - 1) {
                        outputFile << ",";  // Add a comma between values
                    }
                }
                outputFile << "\n";  // Start a new line for each row
            }

            outputFile.close();
            std::cout << "Data has been written to " << filename << std::endl;
        } else {
            std::cerr << "Unable to open file: " << filename << std::endl;
        }
}

// Uncomment for Unit Testing of Code

// int main() {
//     std::vector<double> start = {0, 0, 0};
//     std::vector<double> goal = {7.0*10, 7.0*10, 7.0*10};
//     std::vector<std::vector<double>> space_limits = {{0., 0., 0.9}, {100., 100., 100.}};
//     RRTStar rrt(space_limits, start, goal, 5, 1000, {{15, 55, 15, 55, 15, 55}});
//     rrt.run();

//     std::string filename = "output.csv";
//     rrt.writeDataToCSV(rrt.bestPath, filename);
//     //rrt.plot();
//     return 0;
// }