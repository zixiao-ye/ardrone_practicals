#include <arp/Planner.hpp>

#include "opencv2/highgui/highgui.hpp"

//#include <opencv2/core/mat.hpp>
#include <cmath>


namespace arp{
    //convert point pos into index
    int pos2idx(double pos, int size){
        return std::round(pos/0.1 + double(size-1)/2.0);
    }

    double idx2pos(int size, int idx){
        return (idx-double(size-1)/2.0) * 0.1;
    }

    // Constructor
    Planner::Planner(cv::Mat* map, double start_x, double start_y, double start_z, double dest_x, double dest_y, double dest_z, int *sizes) 
    : mapSizes{sizes}{
        int start_i = pos2idx(start_x, sizes[0]);
        int start_j = pos2idx(start_y, sizes[1]);
        int start_k = pos2idx(start_z, sizes[2]);

        int dest_i = pos2idx(dest_x, sizes[0]);
        int dest_j = pos2idx(dest_y, sizes[1]);
        int dest_k = pos2idx(dest_z, sizes[2]);

        prev = cv::Mat(3, mapSizes, CV_64FC3, cv::Scalar(-1e+30f, -1e+30f, -1e+30f));
        
        start << start_i, start_j, start_k;
        dest << dest_i, dest_j, dest_k; 

        wrappedMapData_ = map;
   
/*         start << 0, 0, 0;
        dest << 2, 0, 0; 
 */    }

    

    //check whether a point is occupied
    bool Planner::isOccupied(int i, int j, int k){
        return (int(wrappedMapData_->at<char>(i,j,k)) > 0);
    }

    // A Utility Function to check whether destination cell has
    // been reached or not
    bool Planner::isDest(Eigen::Vector3i current)
    {
        return current.isApprox(dest, 1e-5f);
    }
    
    // A Utility Function to calculate the 'h' heuristics.
    double Planner::h(Eigen::Vector3i current)
    {
        double weight{10};
        /* for (size_t i = 73; i < current[2]; i++)
        {
            if (isOccupied(current[0], current[1], i)){
                std::cout<<i<<"  below occupied!  "<<current<<std::endl;
                return 1e+29f;
                weight = 100;
            }
        }  */
        
        return weight * euclideanDist(current, dest);
        //return (current-dest).norm();
    }

    double Planner::euclideanDist(Eigen::Vector3i u, Eigen::Vector3i v){
        return sqrt(pow(u[0] - v[0], 2) + pow(u[1] - v[1], 2) + pow(u[2] - v[2], 2));
    }

    //astar algorithm
    void Planner::astar(){
        std::cout<< mapSizes[0]<<"  "<<mapSizes[1]<< "  "<<mapSizes[2]<<std::endl;

        int ii = 207;
        int kk = 89;
        std::cout<<int(wrappedMapData_->at<char>(kk,ii,ii))<<std::endl;

        /* for (int i = 0; i < mapSizes[0]; i++)
        {
            for (int j = 0; j < mapSizes[1]; j++)
            {
                for (int k = 0; k < mapSizes[2]; k++)
                {   
                    //if (i == 207 && j == 207 && k == 89)
                        std::cout<<i<<"  "<<j<<"  "<<k<<" = "<<int(wrappedMapData_->at<char>(k,j,i))<<std::endl;     
                }
            
            }
            
        }  */
        if (isOccupied(dest[0], dest[1], dest[2]))
        {
            std::cout<<"dest is occupied!"<<std::endl;
            return;
        }
        
        int num_vertices = mapSizes[0] * mapSizes[1] * mapSizes[2];

        cv::Mat dist(3, mapSizes, CV_64F, cv::Scalar(1e+30f));
        cv::Mat totDistEst(3, mapSizes, CV_64F, cv::Scalar(1e+30f));
        


        std::vector<Eigen::Vector3i> openSet;
        std::vector<Eigen::Vector3i> closeSet;
        //std::vector<double> distSet;
        openSet.push_back(start);
        dist.at<double>(start[0], start[1], start[2]) = 0;
        totDistEst.at<double>(start[0], start[1], start[2]) = h(start);
        //distSet.push_back(h(start));

        while(!openSet.empty()){  
            
            // vertex in openSet with min totDistEst[u]

            int u_index[3];
            double MinVal;
            minMaxIdx(totDistEst, &MinVal, NULL, u_index, NULL);
            //std::cout<<"Min value in totDistEst: "<<MinVal<<std::endl;

            // remove u from openSet
            Eigen::Vector3i u = {u_index[0], u_index[1], u_index[2]};



            //std::cout<<"U: "<<u[0]<<"  "<<u[1]<<"  "<<u[2]<<" totDistEst: "<<MinVal<<std::endl;
            //std::cout<<"Before erase openSet: "<<openSet.size()<<std::endl;
            openSet.erase(remove(openSet.begin(), openSet.end(), u), openSet.end());
            //std::cout<<openSet.size()<<std::endl;

            closeSet.push_back(u);

            //std::cout<<"check if goal reached"<<std::endl;
            if(isDest(u)){
                std::cout<<"Success, goal reached!"<<std::endl;
                return;
            }
                
                            //return dist.at<double>(current[0], current[1], current[2]), prev; 

          
            //for each neighbour v of u: 
            Eigen::Vector3i v;
            double alt;
            //std::cout<<"go through neighbours"<<std::endl;
            //std::cout<<"Before openSet: "<<openSet.size()<<std::endl;
            for (int k = -2; k < 3; k++){
                //std::cout<<"check"<<std::endl;
                v[2] = u[2] + k;

                //out of boundaries z axis
                if(v[2] > mapSizes[2] || v[2] < 70)
                    continue;

                for (int j = -2; j < 3; j++){

                    v[1] = u[1] + j;

                    //out of boundaries y axis
                    if(v[1] > mapSizes[1] || v[1] < 0)
                        continue;


                    for (int i = -2; i < 3; i++){

                        v[0] = u[0] + i;

                        //out of boundaries x axis  
                        if(v[0] > mapSizes[0] || v[0] < 0)
                            continue;
                            
                        // node himself    
                        if(u.isApprox(v, 1e-9f))
                            continue;

                        // check is not occupied
                        if(isOccupied(v[0], v[1], v[2]))
                            continue;
                        
                        
                        bool flag1 = false;
                        for (int z = 82; z < v[2]; z++)
                        {
                            if (isOccupied(v[0], v[1], z)){
                                std::cout<<z<<"  below occupied!  "<<v<<std::endl;
                                flag1 = true;
                                //continue;
                            }
                        }
                        if (flag1)
                        {
                            continue;
                        }
                        
                        
                        // rule out the parent node    
                        if(std::find(closeSet.begin(),closeSet.end(),v) != closeSet.end())
                            continue;
                        
                        //alt ← dist[u] + G.Edges(u, v).dist
                        //std::cout<<"U: "<<u[0]<<"  "<<u[1]<<"  "<<u[2]<<" V: "<<v[0]<<"  "<<v[1]<<"  "<<v[2]<<std::endl;
                        //std::cout<<"V: "<<v[0]<<"  "<<v[1]<<"  "<<v[2]<<std::endl;
                        alt = dist.at<double>(u[0], u[1], u[2]) + euclideanDist(u, v);
                        //std::cout<<"dist:"<<dist.at<double>(u[0], u[1], u[2])<<"   euclideandist:"<<euclideanDist(u, v)<<std::endl;
                        
                        //if alt < dist[v]: 
                        //std::cout<<"alt:"<<alt<<"   euclideandist:"<<dist.at<double>(v[0], v[1], v[2])<<std::endl;
                        if (alt < dist.at<double>(v[0], v[1], v[2])){
                            /* dist[v] ← alt
                            totDistEst[v] ← alt + h(v, goal)
                            insert v into openSet
                            prev[v] ← u                        
                            */ 
                            dist.at<double>(v[0], v[1], v[2]) = alt;
                            totDistEst.at<double>(v[0], v[1], v[2]) = alt + h(v);
                            //if(std::find(openSet.begin(),openSet.end(),v) == openSet.end())
                            totDistEst.at<double>(u[0], u[1], u[2]) = 1e+30f;
                            
                            openSet.push_back(v);
                            //distSet.push_back(alt + h(v));
                            //prev.at<cv::Scalar>(v[0], v[1], v[2]) = (u[0], u[1], u[2]);
                            prev.at<cv::Scalar>(v[0], v[1], v[2])[0] = u[0];
                            prev.at<cv::Scalar>(v[0], v[1], v[2])[1] = u[1];
                            prev.at<cv::Scalar>(v[0], v[1], v[2])[2] = u[2];
                            //std::cout<<" V: "<<v[0]<<"  "<<v[1]<<"  "<<v[2]<<"dist: "<<alt<<" totDistEst: "<<alt + h(v)<<"  "<<std::endl;
                            //std::cout<<" U: "<<u[0]<<"  "<<u[1]<<"  "<<u[2]<<" prev: "<<prev.at<cv::Scalar>(v[0], v[1], v[2])<<"  "<<std::endl;
                        }                       
                    }
                }
            }

            //std::cout<<"After openSet: "<<openSet.size()<<std::endl;
            
        }


        
    }

    std::deque<Autopilot::Waypoint> Planner::getWaypoints(){
        Eigen::Vector3i u = {dest[0], dest[1], dest[2]};
        std::deque<Autopilot::Waypoint> waypoints;
        int i, j, k;
        double height;
        double threshold = 2;
        
        /* Autopilot::Waypoint waypoint1 = {
                idx2pos(mapSizes[0], u[0]),
                idx2pos(mapSizes[1], u[1]),
                idx2pos(mapSizes[1], u[2]),
                0.0,
                0.1,
        };
        waypoints.push_front(waypoint1);  */
        while(prev.at<cv::Scalar>(u[0], u[1], u[2])[0] > -1e+29f 
                || prev.at<cv::Scalar>(u[0], u[1], u[2])[1] > -1e+29f
                || prev.at<cv::Scalar>(u[0], u[1], u[2])[2] > -1e+29f){
            //std::cout<<"test"<<std::endl;
            height = idx2pos(mapSizes[2], u[2]);
            if (height < threshold)
                height = threshold; 
            if(waypoints.size()==0)
                height = idx2pos(mapSizes[2], u[2]);;
            
            Autopilot::Waypoint waypoint = {
                idx2pos(mapSizes[0], u[0]),
                idx2pos(mapSizes[1], u[1]),
                height,
                0.0,
                0.1,
            };
            waypoints.push_front(waypoint);
            
            i = int(prev.at<cv::Scalar>(u[0], u[1], u[2])[0]);
            j = int(prev.at<cv::Scalar>(u[0], u[1], u[2])[1]);
            k = int(prev.at<cv::Scalar>(u[0], u[1], u[2])[2]);
            //std::cout<<"I J K: "<<i<<"  "<<j<<"  "<<k<<" XYZ: "<<idx2pos(mapSizes[0], u[0])<<"  "<<idx2pos(mapSizes[1], u[1])<<"  "<<idx2pos(mapSizes[2], u[2])<<std::endl;
            //u << int(prev.at<cv::Scalar>(u[0], u[1], u[2])[0]), int(prev.at<cv::Scalar>(u[0], u[1], u[2])[1]), int(prev.at<cv::Scalar>(u[0], u[1], u[2])[2]);
            //u = Eigen::Vector3i(prev.at<cv::Scalar>(u[0], u[1], u[2]));
            /* if(i>mapSizes[0] || j>mapSizes[1] || k>mapSizes[2])
                break; */
            u << i, j, k;
        }
        /* Autopilot::Waypoint waypoint1 = {
                idx2pos(mapSizes[0], u[0]),
                idx2pos(mapSizes[1], u[1]),
                idx2pos(mapSizes[1], u[2]),
                0.2,
                0.1,
        };
        waypoints.push_front(waypoint1); */

        return waypoints;
    }

}