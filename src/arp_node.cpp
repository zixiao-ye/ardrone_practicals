#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>
#include <arp/Frontend.hpp>

//NEW
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"

#include <ros/package.h>
#include "arp/VisualInertialTracker.hpp"
#include "arp/StatePublisher.hpp"
#include "arp/InteractiveMarkerServer.hpp"

#include "arp/Planner.hpp"





class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    //std::cout << "Image shape " << lastImage_.size() << std::endl;

    visualInertialTracker->addImage(timeMicroseconds, lastImage_);
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  Subscriber(arp::VisualInertialTracker *Tracker){
    visualInertialTracker = Tracker;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
    + msg->header.stamp.nsec / 1000;
    Eigen::Vector3d omega_S(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    //omega_S[0] =
    Eigen::Vector3d acc_S(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    visualInertialTracker->addImuMeasurement(timeMicroseconds, omega_S, acc_S);
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
  arp::VisualInertialTracker *visualInertialTracker;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // set up autopilot
  arp::Autopilot autopilot(nh);

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  std::string str1="Instructions: arrow keys for going forward/backward and left/right";
  std::string str2="W/S for going up/down, A/D for yawing left/right";

  double imageWidth, imageHeight, fu, fv, cu, cv, k1, k2, p1, p2, focal_length;
  if (  nh.getParam("/arp_node/fu", fu) && nh.getParam("/arp_node/fv", fv) 
        && nh.getParam("/arp_node/cu", cu) && nh.getParam("/arp_node/cv", cv) 
        && nh.getParam("/arp_node/k1", k1) && nh.getParam("/arp_node/k2", k2) 
        && nh.getParam("/arp_node/p1", p1) && nh.getParam("/arp_node/p2", p2) )
  {
    std::cout<<"Using the next camera parameters: "<<std::endl;
    std::cout<<" - fu: " << fu <<std::endl;
    std::cout<<" - fv: " << fv <<std::endl;
    std::cout<<" - cu: " << cu <<std::endl;
    std::cout<<" - cv: " << cv <<std::endl;
    std::cout<<" - k1: " << k1 <<std::endl;
    std::cout<<" - k2: " << k2 <<std::endl;
    std::cout<<" - p1: " << p1 <<std::endl;
    std::cout<<" - p2: " << p2 <<std::endl;
  }
  else{
    std::cout<<"Fail to get the parameter!"<<std::endl;
  }

  if(nh.getParam("/arp_node/imageWidth", imageWidth) && nh.getParam("/arp_node/imageHeight", imageHeight) ){
    std::cout<<" - imageWidth: " << imageWidth <<std::endl;
    std::cout<<" - imageHeight: " << imageHeight <<std::endl;
  }
  else{
    imageWidth = 640;
    imageHeight = 360;
  }

  if(nh.getParam("/arp_node/focal_length", focal_length)){
    std::cout<<" - focal length for the camera used in building the map: " << focal_length <<std::endl;
  }
  


  //Application integration P3
  // set up frontend -- use parameters as loaded in previous practical
  arp::Frontend frontend(640, 360, fu, fv, cu, cv, k1, k2, p1, p2, focal_length);
  
  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;

  if(!nh.getParam("arp_node/map", mapFile))
    ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;

  if(!frontend.loadMap(mapPath))
    ROS_FATAL_STREAM("could not load map from " << mapPath << " !");


  //load the occupancy map P6
  if(!nh.getParam("arp_node/occupancymap", mapFile))
    ROS_FATAL("error loading parameter");
  std::string filename = path+"/maps/"+mapFile;
  std::ifstream mapFileI(filename, std::ios::in | std::ios::binary);
  if(!mapFileI.is_open()) {
    ROS_FATAL_STREAM("could not open map file " << filename);
  }
  // first read the map size along all the dimensions:
  int sizes[3];
  if(!mapFileI.read((char*)sizes, 3*sizeof(int))) {
    ROS_FATAL_STREAM("could not read map file " << filename);
  }
  // now read the map data: don’t forget to delete[] in the end!

  char* mapData = new char[sizes[0]*sizes[1]*sizes[2]];
  if(!mapFileI.read((char*)mapData, sizes[0]*sizes[1]*sizes[2])) {
    ROS_FATAL_STREAM("could not read map file " << filename);
  }

  mapFileI.close();

  // now wrap it with a cv::Mat for easier access:
  cv::Mat wrappedMapData(3, sizes, CV_8SC1, mapData);
  /* for (int i = 0; i < sizes[0]; i++)
  {
      for (int j = 0; j < sizes[1]; j++)
      {
          for (int k = 0; k < sizes[2]; k++)
          {   
            std::cout<<i<<"  "<<j<<"  "<<k<<" = "<<int(wrappedMapData.at<char>(k,j,i))<<std::endl;     
          }
      
      }
      
  }  */

  Eigen::Vector3d start,dest;
  std::vector<double> dest_array;
  if(!nh.getParam("arp_node/pointB", dest_array))
    ROS_FATAL("error loading destination parameter");
  start << 0, 0, 0;
  dest << dest_array[0], dest_array[1], dest_array[2]; 
  
  // Application P4
  // load DBoW2 vocabulary
  std::string vocPath = path + "/maps/small_voc.yml.gz";
  if(!frontend.loadDBoW2Voc(vocPath))
    ROS_FATAL_STREAM("could not load DBoW2 voc. from " << vocPath << " !");
  frontend.buildDatabase();

  // state publisher -- provided for rviz visualisation of drone pose:
  arp::StatePublisher pubState(nh);

  // set up EKF
  arp::ViEkf viEkf;
  Eigen::Matrix4d T_SC_mat;
  std::vector<double> T_SC_array;
  if(!nh.getParam("arp_node/T_SC", T_SC_array))
    ROS_FATAL("error loading parameter");
  T_SC_mat <<
      T_SC_array[0], T_SC_array[1], T_SC_array[2], T_SC_array[3],
      T_SC_array[4], T_SC_array[5], T_SC_array[6], T_SC_array[7],
      T_SC_array[8], T_SC_array[9], T_SC_array[10], T_SC_array[11],
      T_SC_array[12], T_SC_array[13], T_SC_array[14], T_SC_array[15];
  arp::kinematics::Transformation T_SC(T_SC_mat);
  viEkf.setCameraExtrinsics(T_SC);
  viEkf.setCameraIntrinsics(frontend.camera());

  // set up visual-inertial tracking
  arp::VisualInertialTracker visualInertialTracker;
  visualInertialTracker.setFrontend(frontend);
  visualInertialTracker.setEstimator(viEkf);
  //visualInertialTracker.enableFusion(false);

  // set up visualisation: publish poses to topic ardrone/vi_ekf_pose
  visualInertialTracker.setVisualisationCallback(std::bind(&arp::StatePublisher::publish, &pubState, std::placeholders::_1, std::placeholders::_2));

  // setup inputs
  Subscriber subscriber(&visualInertialTracker);
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  
  // set up interactive marker P5
  double x, y, z, yaw;
  arp::InteractiveMarkerServer server(autopilot);
  double euler_angle_max, control_vz_max, control_yaw;
  if(!nh.getParam("/ardrone_driver/euler_angle_max", euler_angle_max))
    ROS_FATAL("error loading PID limits parameter euler_angle_max");
  if(!nh.getParam("/ardrone_driver/control_vz_max", control_vz_max))
    ROS_FATAL("error loading PID limits parameter control_vz_max");
  if(!nh.getParam("/ardrone_driver/control_yaw", control_yaw))
    ROS_FATAL("error loading PID limits parameter control_yaw");

  visualInertialTracker.setControllerCallback(std::bind(&arp::Autopilot::controllerCallback, &autopilot,
                                                std::placeholders::_1, std::placeholders::_2));
  
  


  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  cv::Mat image;
  cv::Mat undistortImage;
  
  // create the camera model
  arp::cameras::RadialTangentialDistortion distortion = arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera(imageWidth, imageHeight, fu, fv, cu, cv, distortion);
  pinholeCamera.initialiseUndistortMaps(imageWidth, imageHeight, fu,  fv, cu,  cv);
  bool undistort = false;
  bool mode_switch = false;
  bool flyChallenge = false;

  std::string s;
  while (ros::ok()) {

    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }
    

    auto droneStatus = autopilot.droneStatus();

    // render image, if there is a new one available
    if(subscriber.getLastImage(image)) {
      
      //Undistort image
      if (undistort){
        pinholeCamera.undistortImage(image, undistortImage);
        image = undistortImage;
      }

      // TODO: add overlays to the cv::Mat image, e.g. text
      cv::putText(image, 
            "Battery State: "+std::to_string(autopilot.getbatteryPercent()),
            cv::Point(10,20), // Coordinates (Bottom-left corner of the text string in the image)
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.7, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1, // Line Thickness (Optional)
            cv:: LINE_AA); // Anti-alias (Optional, see version note)

      cv::putText(image, 
            "Current Ardrone Status: "+std::to_string(droneStatus),
            cv::Point(350,20), // Coordinates (Bottom-left corner of the text string in the image)
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.7, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1, // Line Thickness (Optional)
            cv:: LINE_AA); // Anti-alias (Optional, see version note)



      cv::putText(image, 
            str1,
            cv::Point(0,300), // Coordinates (Bottom-left corner of the text string in the image)
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.7, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1, // Line Thickness (Optional)
            cv:: LINE_AA); // Anti-alias (Optional, see version note)

      cv::putText(image, 
            str2,
            cv::Point(130,320), // Coordinates (Bottom-left corner of the text string in the image)
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.7, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1, // Line Thickness (Optional)
            cv:: LINE_AA); // Anti-alias (Optional, see version note)

      cv::putText(image, 
            "U for undistort, I for distort image",
            cv::Point(130,340), // Coordinates (Bottom-left corner of the text string in the image)
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.7, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1, // Line Thickness (Optional)
            cv:: LINE_AA); // Anti-alias (Optional, see version note)

            
      // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
      // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
      // remember to pick the right SDL_PIXELFORMAT_* !
      texture = SDL_CreateTexture(
          renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
      SDL_UpdateTexture(texture, NULL, (void*)image.data, image.step1());
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, NULL, NULL);
      SDL_RenderPresent(renderer);
      // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
      SDL_DestroyTexture(texture);
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // check states!
    //auto droneStatus = autopilot.droneStatus();
    // command
    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_U]) {
      std::cout << "Changing image to undistort" << std::endl;
      undistort = true;
    }

    if (state[SDL_SCANCODE_I]) {
      std::cout << "Changing image to distort" << std::endl;
      undistort = false;
    }

    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    //switch between automatic mode and manual mode
    if (state[SDL_SCANCODE_RCTRL]) {
      std::cout << "Enabling automatic control..." << std::endl;
      autopilot.setAutomatic();
      mode_switch = true;
    }
    if (state[SDL_SCANCODE_SPACE]) {
      std::cout << "Switching back to manual control..." << std::endl;
      flyChallenge = false;
      autopilot.setManual();
    }
    if (state[SDL_SCANCODE_P]) {
      flyChallenge = true;
      autopilot.setAutomatic();
      std::cout << "Switching into the Challenge Mode..." << std::endl;
      if(autopilot.haspointA() && autopilot.waypointsLeft() == 0&& autopilot.waypoints_rhLeft() == 0){
        start = autopilot.getpointA();
        //initialize the planner
        arp::Planner planner(&wrappedMapData, start[0], start[1], start[2], dest[0], dest[1], dest[2], sizes);
        // autopilot.activatePlanner(planner);
        planner.astar();
        autopilot.flyPath(planner.getWaypoints());
        std::cout << "Taking off...                          status=" << droneStatus;
        bool success = autopilot.takeoff();
        if (success) {
          std::cout << " [ OK ]" << std::endl;
        } else {
          std::cout << " [FAIL]" << std::endl;
        }
      }
    }
  
    if(!autopilot.isAutomatic()){
      // TODO: process moving commands when in state 3,4, or 7
      double forward=0;
      double left=0;
      double up=0;
      double rotateLeft=0;

      if (state[SDL_SCANCODE_UP]) {
        std::cout << "Moving the drone forward...     status=" << droneStatus;
        forward+=0.5;
      }
      if (state[SDL_SCANCODE_DOWN]) {
        std::cout << "Moving the drone backward...     status=" << droneStatus;
        forward+=-0.5;
      }
      if (state[SDL_SCANCODE_LEFT]) {
        std::cout << "Moving the drone left...     status=" << droneStatus;
        left+=0.5;
      }
      if (state[SDL_SCANCODE_RIGHT]) {
        std::cout << "Moving thr drone right...     status=" << droneStatus;
        left+=-0.5;
      }
      
      if (state[SDL_SCANCODE_W]) {
        std::cout << "Moving the drone up...     status=" << droneStatus;
        up+=0.5;
      }
      if (state[SDL_SCANCODE_S]) {
        std::cout << "Moving the drone down...     status=" << droneStatus;
        up+=-0.5;
      }
      if (state[SDL_SCANCODE_A]) {
        std::cout << "Yawing the drone left...     status=" << droneStatus;
        rotateLeft += 0.5;
      }
      if (state[SDL_SCANCODE_D]) {
        std::cout << "Yawing the drone right...     status=" << droneStatus;
        rotateLeft += -0.5;
      }      
      bool success = autopilot.manualMove(forward,left,up,rotateLeft);
      if ((forward != 0 || left!=0 || up !=0 || rotateLeft != 0) && success) {
        std::cout << " [ OK ] " << std::endl;
      } 
      if ((forward != 0 || left!=0 || up !=0 || rotateLeft != 0) && !success) {
        std::cout << " [FAIL] " << std::endl;
      }    
    }
    else{
      if (mode_switch)
      {
        autopilot.getPoseReference(x, y, z, yaw);
        //server.activate(x, y, z, yaw);
        mode_switch = false;
      }
      if (flyChallenge)
      {
        /* if (droneStatus == arp::Autopilot::Landed)
        {
          std::cout << "Taking off...                          status=" << droneStatus;
          bool success = autopilot.takeoff();
          if (success) {
            std::cout << " [ OK ]" << std::endl;
          } else {
            std::cout << " [FAIL]" << std::endl;
          }
        }  */

        /* if (droneStatus == arp::Autopilot::Flying || droneStatus == arp::Autopilot::Flying2 || droneStatus == arp::Autopilot::Hovering )
        {
          std::cout << "Going to land...                       status=" << droneStatus;
          bool success = autopilot.land();
          if (success) {
            std::cout << " [ OK ]" << std::endl;
          } else {
            std::cout << " [FAIL]" << std::endl;
          }
        }  */
      
        
      }
      
    }
  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

