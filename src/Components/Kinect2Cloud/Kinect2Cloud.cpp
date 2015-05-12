/*!
 * \file
 * \brief
 * \author Dawid Kaczmarek
 */

#include <memory>
#include <string>

#include "Kinect2Cloud.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/filter.h>

namespace Processors {
namespace Kinect2Cloud {

Kinect2Cloud::Kinect2Cloud(const std::string & name) :
        Base::Component(name)
{
    //registerProperty(enable_rgb);
    //registerProperty(enable_ir);
    //registerProperty(enable_depth);
}

Kinect2Cloud::~Kinect2Cloud() {

}

void Kinect2Cloud::prepareInterface() {   
    registerStream("in_rgb_image", &in_rgb_image);
    registerStream("in_disparity_image", &in_disparity_image);
    registerStream("in_ir_camera_matrix", &in_ir_camera_matrix);
    registerStream("in_rgb_camera_matrix", &in_rgb_camera_matrix);
    registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
    registerStream("out_rgb_image", &out_rgb_image);
    registerStream("out_xyz_image", &out_xyz_image);


	// Register handlers
    h_calculateCloud.setup(boost::bind(&Kinect2Cloud::calculateCloud, this));
    registerHandler("calculateCloud", &h_calculateCloud);
    addDependency("calculateCloud",&in_rgb_image);
    addDependency("calculateCloud",&in_disparity_image);
    addDependency("calculateCloud",&in_ir_camera_matrix);
    addDependency("calculateCloud",&in_rgb_camera_matrix);
}

bool Kinect2Cloud::onInit() {
    return true;
}

bool Kinect2Cloud::onFinish() {
	return true;
}

bool Kinect2Cloud::onStop() {
	return true;
}

bool Kinect2Cloud::onStart() {
	return true;
}


void Kinect2Cloud::calculateCloud() {
    CLOG(LINFO) << "Getting data";

    color = in_rgb_image.read();
    depth = in_disparity_image.read();
    cv::Mat xyz(depth.rows, depth.cols, CV_8UC3);

    rgbCamInfo = in_rgb_camera_matrix.read();
    irCamInfo = in_ir_camera_matrix.read();

    cameraMatrixColor = rgbCamInfo.cameraMatrix();

    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    createCloud(depth, color, cloud, xyz);

    out_cloud_xyzrgb.write(cloud);
    out_xyz_image.write(xyz);
}

void Kinect2Cloud::createLookup(size_t width, size_t height)
{
  const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
  const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
  const float cx = cameraMatrixColor.at<double>(0, 2);
  const float cy = cameraMatrixColor.at<double>(1, 2);
  float *it;

  lookupY = cv::Mat(1, height, CV_32F);
  it = lookupY.ptr<float>();
  for(size_t r = 0; r < height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  lookupX = cv::Mat(1, width, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}

void Kinect2Cloud::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat &xyz) const
{
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  #pragma omp parallel for
  for(int r = 0; r < depth.rows; ++r)
  {
    pcl::PointXYZRGB *itP = &cloud->points[r * depth.cols];
    const uint16_t *itD = depth.ptr<uint16_t>(r);
          cv::Vec3b *itDo = xyz.ptr<cv::Vec3b>(r);
    const cv::Vec3b *itC  = color.ptr<cv::Vec3b>(r);
    const float y = lookupY.at<float>(0, r);
    const float *itX = lookupX.ptr<float>();

    for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itDo, ++itC, ++itX)
    {
      register const float depthValue = *itD / 1000.0f;
      // Check for invalid measurements
      if(isnan(depthValue) || depthValue <= 0.001)
      {
        // not valid
        itP->x = itP->y = itP->z = badPoint;
        itP->rgb = 0;
        continue;
      }
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
      itP->b = itC->val[0];
      itP->g = itC->val[1];
      itP->r = itC->val[2];
      itDo->val[0] = static_cast<char>(itP->x);
      itDo->val[1] = static_cast<char>(itP->y);
      itDo->val[2] = static_cast<char>(itP->z);
      //itP->a = 0;
    }
  }
}



} //: namespace Kinect2Device
} //: namespace Processors


