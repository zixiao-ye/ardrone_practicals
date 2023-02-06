/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>

#include "arp/kinematics/Transformation.hpp"

#include <chrono>
#include <thread>


namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode  

  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);

  pubMove_ =nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);
  
  PidController::Parameters p;
  p.k_p = 1;
  p.k_i = 0.08;
  p.k_d = 0.1;
  pid_x.setParameters(p);
  pid_y.setParameters(p);

  p.k_p = 1.0;
  p.k_i = 0.05;
  p.k_d = 0.05;
  pid_z.setParameters(p);

  p.k_p = 1.5;
  p.k_i = 0;
  p.k_d = 0;
  pid_yaw.setParameters(p);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  return move(forward, left, up, rotateLeft); 

}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{
  // TODO: implement...
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping)
  {
    geometry_msgs::Twist moveMsg;
    moveMsg.linear.x=forward;
    moveMsg.linear.y=left;
    moveMsg.linear.z=up;
    moveMsg.angular.z=rotateLeft;

    pubMove_.publish(moveMsg);

    return true;
  }
  


  return false;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

/* void Autopilot::activatePlanner(arp::Planner planner){
  //planner.astar();
} */

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{
  
  const double yaw = kinematics::yawAngle(x.q_WS);
  // only do anything here, if automatic
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    return;
  }
  
  // TODO: only enable when in flight
  DroneStatus dronestatus = droneStatus();
  
  if (dronestatus == Landing || dronestatus == TakingOff)
    return;
  
  if(!pointA_flag){
    pointA[0] = x.t_WS[0];
    pointA[1] = x.t_WS[1];
    pointA[2] = x.t_WS[2];
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAA"<<pointA<<std::endl;
    pointA_flag = true;
  }

  //std::cout<<"test"<<std::endl;
  if(!waypoints_.empty()){
    // TODO: setPoseReference() from current waypoint
    setPoseReference(waypoints_[0].x, waypoints_[0].y, waypoints_[0].z, waypoints_[0].yaw);
    //posErr = sqrt(pow(u[0] - v[0], 2) + pow(u[1] - v[1], 2) + pow(u[2] - v[2], 2)); 
  }else if(!waypoints_rh.empty()){
    setPoseReference(waypoints_rh[0].x, waypoints_rh[0].y, waypoints_rh[0].z, waypoints_rh[0].yaw);
  }else if(dronestatus == Flying || dronestatus == Flying2 || dronestatus == Hovering){  // Landing at waypoint A
    std::cout << "Going to land...Landing at waypoint A                       status=";
    bool success = land();
    if (success) {
      std::cout << " [ OK ]" << std::endl;
    } else {
      std::cout << " [FAIL]" << std::endl;
    }
    return;
  }
  std::cout<<waypoints_.size()<<" "<<waypoints_rh.size()<<" "<<dronestatus<<std::endl;
  if(waypoints_.empty() && !waypoints_rh.empty() && dronestatus == Landed)
  {
      std::cout << "Taking off...                          status=";
      bool success = takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      return;
  }
  if(waypoints_.empty() && (dronestatus == Flying || dronestatus == Flying2 || dronestatus == Hovering) &&  arrive_)  // Landing at waypoint B
  {
      std::cout << "Going to land...Landing at waypoint B                       status=";
      bool success = land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(4000));
      arrive_ = false;
      return;
  }
  
  
  kinematics::Transformation T_WS(x.t_WS, x.q_WS);
  Eigen::Matrix3d R_SW = T_WS.R().transpose();

  Eigen::Vector3d ref_pos; 
  ref_pos<< ref_x_, ref_y_, ref_z_;
  Eigen::Vector3d err_pos = R_SW * (ref_pos - x.t_WS);

  std::lock_guard<std::mutex> l(refMutex_);
  if(!waypoints_.empty()){
    if(err_pos.squaredNorm() < waypoints_[0].posTolerance){
      waypoints_rh.push_front(waypoints_[0]);
      waypoints_.pop_front();
      if (waypoints_.empty())
        arrive_ = true;
    }
  }else if (!waypoints_rh.empty())
  {
    if(err_pos.squaredNorm() < waypoints_rh[0].posTolerance){
      waypoints_rh.pop_front();
    }
  }
  

  std::cout<<"Ref pos: "<<ref_pos<<" Drone pos: "<<x.t_WS<<std::endl;

  double err_yaw = ref_yaw_ - yaw;
  if(abs(err_yaw) > M_PI){
    err_yaw = -M_PI + fmod(2*M_PI + fmod(err_yaw + M_PI, 2*M_PI), 2*M_PI);
  }

  Eigen::Vector3d err_pos_dot = -R_SW * x.v_W;
  double err_yaw_dot = 0;

  // TODO: get ros parameter
  double euler_angle_max, control_vz_max, control_yaw;
  if(!nh_->getParam("/ardrone_driver/euler_angle_max", euler_angle_max) || !nh_->getParam("/ardrone_driver/control_vz_max", control_vz_max) || !nh_->getParam("/ardrone_driver/control_yaw", control_yaw))
    ROS_FATAL("error loading PID limits parameters"); 

  control_vz_max *= pow(10, -3);

  // TODO: compute control output
  pid_x.setOutputLimits(-euler_angle_max, euler_angle_max);
  pid_y.setOutputLimits(-euler_angle_max, euler_angle_max);
  pid_z.setOutputLimits(-control_vz_max, control_vz_max);
  pid_yaw.setOutputLimits(-control_yaw, control_yaw);

  double forward, left, up, rotateleft;
  forward = pid_x.control(timeMicroseconds, err_pos(0), err_pos_dot(0));
  left = pid_y.control(timeMicroseconds, err_pos(1), err_pos_dot(1));
  up = pid_z.control(timeMicroseconds, err_pos(2), err_pos_dot(2));
  rotateleft = pid_yaw.control(timeMicroseconds, err_yaw, err_yaw_dot);

  // TODO: send to move
  //std::cout<<forward<<" "<<left<<" "<<up<<" "<<rotateleft<<std::endl;
  move(forward, left, up, rotateleft);

}

}  // namespace arp

