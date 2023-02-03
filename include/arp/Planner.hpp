#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_PLANNER_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_PLANNER_HPP_

#include <opencv2/core/mat.hpp>
#include <cmath>

#include <arp/Autopilot.hpp>

namespace arp{

class Planner{
public:
    // Constructor
    Planner(cv::Mat& map, double start_x, double start_y, double start_z, double dest_x, double dest_y, double dest_z, int sizes[3]);

    //check whether a point is occupied
    bool isOccupied(int i, int j, int k);

    // A Utility Function to check whether destination cell has
    // been reached or not
    bool isDest(Eigen::Vector3i current);
    
    // A Utility Function to calculate the 'h' heuristics.
    double h(Eigen::Vector3i current);

    //astar algorithm
    void astar();

    //calculate euclidean diatance between two points
    double euclideanDist(Eigen::Vector3i u, Eigen::Vector3i v);

    std::deque<Autopilot::Waypoint> getWaypoints();

   
protected:
    cv::Mat* wrappedMapData_;
    cv::Mat prev;
    int* mapSizes;
    Eigen::Vector3i start;
    Eigen::Vector3i dest;
};

int pos2idx(double pos, int size);

double idx2pos(int size, int idx);

}

#endif