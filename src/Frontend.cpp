/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2020
 *      Author: sleutene
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>


#include <brisk/brisk.h>

#include <arp/Frontend.hpp>

#ifndef CV_AA
#define CV_AA cv::LINE_AA // maintains backward compatibility with older OpenCV
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY // maintains backward compatibility with older OpenCV
#endif

namespace arp {

Frontend::Frontend(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2, double focalLength) :
  camera_(imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV,
          arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2))
{
  camera_.initialiseUndistortMaps();

  // also save for OpenCV RANSAC later
  cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cameraMatrix_.at<double>(0,0) = focalLengthU;
  cameraMatrix_.at<double>(1,1) = focalLengthV;
  cameraMatrix_.at<double>(0,2) = imageCenterU;
  cameraMatrix_.at<double>(1,2) = imageCenterV;
  cameraMatrix_.at<double>(2,2) = 1.0;
  distCoeffs_ = cv::Mat::zeros(1, 4, CV_64FC1);
  distCoeffs_.at<double>(0) = k1;
  distCoeffs_.at<double>(1) = k2;
  distCoeffs_.at<double>(2) = p1;
  distCoeffs_.at<double>(3) = p2;
  
  // BRISK detector and descriptor
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(5, 2, 50, 750)); //Original values: 10, 0, 100, 2000
  extractor_.reset(new brisk::BriskDescriptorExtractor(true, false));
  
  // leverage camera-aware BRISK (caution: needs the *_new* maps...)
  cv::Mat rays = cv::Mat(imageHeight, imageWidth, CV_32FC3);
  cv::Mat imageJacobians = cv::Mat(imageHeight, imageWidth, CV_32FC(6));
  for (int v=0; v<imageHeight; ++v) {
    for (int u=0; u<imageWidth; ++u) {
      Eigen::Vector3d ray;
      Eigen::Matrix<double, 2, 3> jacobian;
      if(camera_.backProject(Eigen::Vector2d(u,v), &ray)) {
        ray.normalize();
      } else {
        ray.setZero();
      }
      rays.at<cv::Vec3f>(v,u) = cv::Vec3f(ray[0],ray[1],ray[2]);
      Eigen::Vector2d pt;
      if(camera_.project(ray, &pt, &jacobian)
         ==cameras::ProjectionStatus::Successful) {
        cv::Vec6f j;
        j[0]=jacobian(0,0);
        j[1]=jacobian(0,1);
        j[2]=jacobian(0,2);
        j[3]=jacobian(1,0);
        j[4]=jacobian(1,1);
        j[5]=jacobian(1,2);
        imageJacobians.at<cv::Vec6f>(v,u) = j;
      }
    }
  }
  std::static_pointer_cast<cv::BriskDescriptorExtractor>(extractor_)->setCameraProperties(rays, imageJacobians, focalLength);
}

bool  Frontend::loadMap(std::string path) {
  std::ifstream mapfile(path);
  if(!mapfile.good()) {
    return false;
  }
  
  // read each line
  std::string line;
  std::set<uint64_t> lmIds;
  uint64_t poseId = 0;
  LandmarkVec landmarks;
  while (std::getline(mapfile, line)) {

    // Convert to stringstream
    std::stringstream ss(line);
    
    if(0==line.compare(0, 7,"frame: ")) {
      // store previous set into map
      landmarks_[poseId] = landmarks;
      // get pose id:
      std::stringstream frameSs(line.substr(7,line.size()-1));
      frameSs >> poseId;
      if(!frameSs.eof()) {
        std::string covisStr;
        frameSs >> covisStr; // comma
        frameSs >> covisStr;
        if(0==covisStr.compare("covisibilities:")) {
          while(!frameSs.eof()) {
            uint64_t covisId;
            frameSs >> covisId;
            covisibilities_[poseId].insert(covisId);
          }
        }
      }
      // move to filling next set of landmarks
      landmarks.clear();
    } else {
      if(poseId>0) {
      
        // get keypoint idx
        size_t keypointIdx;
        std::string keypointIdxString;
        std::getline(ss, keypointIdxString, ',');
        std::stringstream(keypointIdxString) >> keypointIdx;
        
        // get landmark id
        uint64_t landmarkId;
        std::string landmarkIdString;
        std::getline(ss, landmarkIdString, ',');
        std::stringstream(landmarkIdString) >> landmarkId;
        
        // read 3d position
        Landmark landmark;
        for(int i=0; i<3; ++i) {
          std::string coordString;
          std::getline(ss, coordString, ',');
          double coord;
          std::stringstream(coordString) >> coord;
          landmark.point[i] = coord;
        }

        // Get descriptor
        std::string descriptorstring;
        std::getline(ss, descriptorstring);
        landmark.descriptor = cv::Mat(1,48,CV_8UC1);
        for(int col=0; col<48; ++col) {
          uint32_t byte;
          std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
          landmark.descriptor.at<uchar>(0,col) = byte;
        }
        lmIds.insert(landmarkId);
        landmarks.push_back(landmark);
      }      
    } 
  }
  if(poseId>0) {
    // store into map
    landmarks_[poseId] = landmarks;
  }
  std::cout << "loaded " << lmIds.size() << " landmarks from " << landmarks_.size() << " poses." << std::endl;
  return lmIds.size() > 0;
}

bool Frontend::loadDBoW2Voc(std::string path) {
  std::cout << "Loading DBoW2 vocabulary from " << path << std::endl;
  dBowVocabulary_.load(path);
  // Hand over vocabulary to dataset. false = do not use direct index:
  dBowDatabase_.setVocabulary(dBowVocabulary_, false, 0);
  std::cout << "loaded DBoW2 vocabulary with " << dBowVocabulary_.size() << " words." << std::endl;
  return true; 
}

bool Frontend::buildDatabase() {
  // add landmarks to the database
  DBoW2::QueryResults ret;
  for(const auto& lms:landmarks_)
  {
    auto ID = lms.first;
    std::vector<DBoW2::FBrisk::TDescriptor> features;
    for (const auto& lm:lms.second) // go through all landmarks seen from this pose
    { 
      //<std::vector<cv::Mat> > vec;
      cv::Mat copy = lm.descriptor.clone();
      //uchar* descriptor = copy.data;
      features.push_back(copy);
    }
    auto index = dBowDatabase_.add(features);
    db2kf.insert({index, ID}); //Matching the database_ID and the keyframe_ID
    //std::cout<<index<<ID<<std::endl;
    /* dBowDatabase_.query(features, ret, -1);
    std::cout << "Searching for Keyframe:: "<< ret<<std::endl; */
  }
  /* for (size_t i = 0; i < db2kf.size(); i++)
  {
    std::cout<<i<<db2kf[i]<<std::endl;
  } */

  /* for(const auto& lms : landmarks_) { // go through all poses
    std::cout<<lms.first<<std::endl;
  } */
  
  

  return true; 
}

int Frontend::detectAndDescribe(
    const cv::Mat& grayscaleImage, const Eigen::Vector3d& extractionDirection,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const {

  // run BRISK detector
  detector_->detect(grayscaleImage, keypoints);

  // run BRISK descriptor extractor
  // orient the keypoints according to the extraction direction:
  Eigen::Vector3d ep;
  Eigen::Vector2d reprojection;
  Eigen::Matrix<double, 2, 3> Jacobian;
  Eigen::Vector2d eg_projected;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    cv::KeyPoint& ckp = keypoints[k];
    const Eigen::Vector2d kp(ckp.pt.x, ckp.pt.y);
    // project ray
    camera_.backProject(kp, &ep);
    // obtain image Jacobian
    camera_.project(ep+extractionDirection.normalized()*0.001, &reprojection);
    // multiply with gravity direction
    eg_projected = reprojection-kp;
    double angle = atan2(eg_projected[1], eg_projected[0]);
    // set
    ckp.angle = angle / M_PI * 180.0;
  }
  extractor_->compute(grayscaleImage, keypoints, descriptors);

  return keypoints.size();
}

bool Frontend::ransac(const std::vector<cv::Point3d>& worldPoints, 
                      const std::vector<cv::Point2d>& imagePoints, 
                      kinematics::Transformation & T_CW, std::vector<int>& inliers) const {
  if(worldPoints.size() != imagePoints.size()) {
    return false;
  }
  if(worldPoints.size()<5) {
    return false; // not realiabl enough
  }

  inliers.clear();
  cv::Mat rvec, tvec;
  bool ransacSuccess = cv::solvePnPRansac(
      worldPoints, imagePoints, cameraMatrix_, distCoeffs_,
      rvec, tvec, false, 100, 5.0, 0.99, inliers, cv::SOLVEPNP_EPNP);	

  // set pose
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T_CW_mat = Eigen::Matrix4d::Identity();
  for(int i=0; i<3; i++) {
    T_CW_mat(i,3) = tvec.at<double>(i);
    for(int j=0; j<3; j++) {
      T_CW_mat(i,j) = R.at<double>(i,j);
    }
  }
  T_CW = kinematics::Transformation(T_CW_mat);

  return ransacSuccess && (double(inliers.size())/double(imagePoints.size()) > 0.6);
}

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage, bool needsReInitialisation)
{
  detections.clear(); // make sure empty

  // to gray:
  cv::Mat grayScale;
  cv::cvtColor(image, grayScale, CV_BGR2GRAY);

  // run BRISK detector and descriptor extractor:
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detectAndDescribe(grayScale, extractionDirection, keypoints, descriptors);

  // TODO match to map:
  const int numPosesToMatch = 3; //P3 check only first 3 frames
  int checkedPoses = 0; //P3

  std::vector<DBoW2::FBrisk::TDescriptor> features;
  DBoW2::QueryResults ret;

  //int rows = descriptors.rows;
  //int cols = descriptors.cols;
  //std::cout<<rows<<" "<<cols<<std::endl;

  for(size_t k = 0; k < keypoints.size(); ++k){
    //uchar* keypointDescriptor = descriptors.data + k*48;
    cv::Mat copy = descriptors.row(k).clone();
    features.push_back(copy);
  }


  if (needsReInitialisation){
    int num_ret = 3; // Number of top query results
    std::vector<uint64_t> query_frames_;
    dBowDatabase_.query(features, ret, num_ret);

    for (size_t i = 0; i < num_ret; i++){
      query_frames_.push_back(db2kf[ret[i].Id]);
    }
    //std::cout << "Searching for Keyframe:: "<< ret<<std::endl;

    int max_matches = 0;
    for(const auto& kf_id : query_frames_){ // go through query frames
      int num_matches = 0;
      for(const auto lm : landmarks_[kf_id]) { // go through all landmarks seen from this pose
        for(size_t k = 0; k < keypoints.size(); ++k) { // go through all keypoints in the frame
          uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
          const float dist = brisk::Hamming::PopcntofXORed(
                keypointDescriptor, lm.descriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)
          // TODO check if a match and process accordingly
          if (dist < 60)
            num_matches++;
        }
      }
      if (num_matches > max_matches){
        max_matches = num_matches;
        active_fm_id = kf_id;
      }
    } 
  }
  
  std::vector<cv::Point3d> worldPoints;
  std::vector<cv::Point2d> imagePoints;
  DetectionVec detections_copy;

  int max_distance = 200; // Frame id gap, we choose the nearest frames
  int max_covisible = 60; // Number of covisibles frames
  
  std::set<uint64_t> covisibles;
  covisibles.insert(active_fm_id);
  for(auto itr:covisibilities_[active_fm_id]){
    int dist = (active_fm_id > itr) ? active_fm_id - itr :  itr - active_fm_id;
    if (dist < max_distance && covisibles.size() < max_covisible){
      covisibles.insert(itr);  
    }
    if (covisibles.size() >= max_covisible){
      break;
    }
  }

  int max_matches = 0;
  for(const auto& likely_frame_id : covisibles){ // go through active keyframes + covisibles
    int num_matches = 0;      
    for(const auto lm : landmarks_[likely_frame_id]) { // go through all landmarks seen from this pose
      for(size_t k = 0; k < keypoints.size(); ++k) { // go through all keypoints in the frame
        uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
        const float dist = brisk::Hamming::PopcntofXORed(
              keypointDescriptor, lm.descriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)
        // TODO check if a match and process accordingly
        if (dist < 60){
          num_matches++;
          cv::KeyPoint& ckp = keypoints[k];
          Eigen::Vector2d kp(ckp.pt.x, ckp.pt.y);
          cv::Point2d imagePoint(ckp.pt.x, ckp.pt.y);
          cv::Point3d worldPoint(lm.point[0], lm.point[1], lm.point[2]);            
          if (std::find(imagePoints.begin(), imagePoints.end(), imagePoint) == imagePoints.end())
          {
            Detection detection;
            detection.keypoint = kp;
            detection.landmark = lm.point;
            detection.landmarkId = lm.landmarkId;
            imagePoints.push_back(imagePoint);
            worldPoints.push_back(worldPoint);
            detections_copy.push_back(detection);
          }
        }
      }
    }
    if (num_matches > max_matches){
      max_matches = num_matches;
      active_fm_id = likely_frame_id;
    }
  }
  

  // TODO run RANSAC (to remove outliers and get pose T_CW estimate)

  std::vector<int> inliers;
  bool ransacSuccess = ransac(worldPoints, imagePoints, T_CW, inliers);
  //std::cout<<"The value of needsReInitialisation: "<<needsReInitialisation<<" The value of ransacSuccess: "<<ransacSuccess<<std::endl;

  // TODO set detections
  for (size_t i = 0; i < inliers.size(); i++)
  {
    detections.push_back(detections_copy[inliers[i]]);
  }
  
  visualisationImage = image;
  // TODO visualise by painting stuff into visualisationImage
  for (size_t i = 0; i < imagePoints.size(); i++)
  {
    if (std::find(inliers.begin(), inliers.end(), i) != inliers.end())
    {
      //std::cout<<"matching"<<std::endl;
      cv::circle(visualisationImage, imagePoints[i], 8, cv::Scalar(0, 255, 0));
    }
    else
    {
      //std::cout<<"not matching"<<std::endl;
      cv::circle(visualisationImage, imagePoints[i], 8, cv::Scalar(0, 0, 255));
    }    
  }
  
  
  
  return ransacSuccess || needsReInitialisation; // TODO return true if successful...
}

}  // namespace arp

