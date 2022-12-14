// Bring in my package's API, which is what I'm testing
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>

// Test the projection and unprojection
TEST(PinholeCamera, projectBackProject)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();
/*   std::cout << "TEST point_C: \n" << point_C << std::endl;
 */
  // project
  Eigen::Vector2d imagePoint;
  pinholeCamera.project(point_C,&imagePoint);
/*   std::cout << "TEST imagePoint: \n" << imagePoint << std::endl;
 */
  // backProject
  Eigen::Vector3d ray_C;
  pinholeCamera.backProject(imagePoint,&ray_C);
/*   std::cout << "TEST ray_C: \n" << ray_C << std::endl;
 */
  // now they should align:
  EXPECT_TRUE(fabs(ray_C.normalized().transpose()*point_C.normalized()-1.0)<1.0e-10);
}

// TODO: write more tests here...

// Test the Jacobian
 TEST(PinholeCamera, jacobians)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random image point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();
  
  double delta = 1e-5;

  Eigen::Matrix<double, 2, 3> pointJacobian;
  Eigen::Matrix<double, 2, 3> numeric_pointJacobian;

  Eigen::Vector3d point1;
  point1[0] = point_C[0] + delta;
  point1[1] = point_C[1];
  point1[2] = point_C[2];

  Eigen::Vector3d point2;
  point2[0] = point_C[0] - delta;
  point2[1] = point_C[1];
  point2[2] = point_C[2];

  Eigen::Vector3d point3;
  point3[0] = point_C[0];
  point3[1] = point_C[1] + delta;
  point3[2] = point_C[2];

  Eigen::Vector3d point4;
  point4[0] = point_C[0];
  point4[1] = point_C[1] - delta;
  point4[2] = point_C[2];

  Eigen::Vector3d point5;
  point5[0] = point_C[0];
  point5[1] = point_C[1];
  point5[2] = point_C[2] + delta;

  Eigen::Vector3d point6;
  point6[0] = point_C[0];
  point6[1] = point_C[1];
  point6[2] = point_C[2] - delta;


  Eigen::Vector2d imagePoint1;
  Eigen::Vector2d imagePoint2;
  Eigen::Vector2d imagePoint3;
  Eigen::Vector2d imagePoint4;
  Eigen::Vector2d imagePoint5;
  Eigen::Vector2d imagePoint6;

  Eigen::Vector2d imagePoint;
  pinholeCamera.project(point_C,&imagePoint,&pointJacobian);

  pinholeCamera.project(point1,&imagePoint1);
  pinholeCamera.project(point2,&imagePoint2);

  pinholeCamera.project(point3,&imagePoint3);
  pinholeCamera.project(point4,&imagePoint4);

  pinholeCamera.project(point5,&imagePoint5);
  pinholeCamera.project(point6,&imagePoint6);

  numeric_pointJacobian   << (imagePoint1[0]-imagePoint2[0])/(2*delta), (imagePoint3[0]-imagePoint4[0])/(2*delta), (imagePoint5[0]-imagePoint6[0])/(2*delta),
                             (imagePoint1[1]-imagePoint2[1])/(2*delta), (imagePoint3[1]-imagePoint4[1])/(2*delta), (imagePoint5[1]-imagePoint6[1])/(2*delta);

  Eigen::Vector3d ray_C;
  pinholeCamera.backProject(imagePoint,&ray_C);

  // now they should align:
  EXPECT_TRUE((pointJacobian - numeric_pointJacobian).norm()<1.0e-6);
} 


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

