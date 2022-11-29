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

//NEW
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"


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

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

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

  double imageWidth, imageHeight, fu, fv, cu, cv, k1, k2, p1, p2;
  if (nh.getParam("/arp_node/imageWidth", imageWidth) && nh.getParam("/arp_node/imageHeight", imageHeight) 
        &&nh.getParam("/arp_node/fu", fu) && nh.getParam("/arp_node/fv", fv) 
        && nh.getParam("/arp_node/cu", cu) && nh.getParam("/arp_node/cv", cv) 
        && nh.getParam("/arp_node/k1", k1) && nh.getParam("/arp_node/k2", k2) 
        && nh.getParam("/arp_node/p1", p1) && nh.getParam("/arp_node/p2", p2) )
  {
    std::cout<<"Using the next camera parameters: "<<std::endl;
    std::cout<<" - imageWidth: " << imageWidth <<std::endl;
    std::cout<<" - imageHeight: " << imageHeight <<std::endl;
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

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  cv::Mat image;
  cv::Mat undistortImage;
  
  // create the camera model
  arp::cameras::RadialTangentialDistortion distortion = arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera(imageWidth, imageHeight, fu, fv, cu, cv, distortion);
  pinholeCamera.initialiseUndistortMaps(imageWidth, imageHeight, fu,  fv, cu,  cv);

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
      pinholeCamera.undistortImage(image, undistortImage);
      image = undistortImage;

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

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

