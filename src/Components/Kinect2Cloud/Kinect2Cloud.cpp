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
    registerStream("out_disparity_image", &out_disparity_image);


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
    cv::Mat bigRgbImage = in_rgb_image.read();
    cv::Mat smallDispMap = in_disparity_image.read();
    Types::CameraInfo rgbCamInfo = in_rgb_camera_matrix.read();
    Types::CameraInfo irCamInfo = in_ir_camera_matrix.read();

    CLOG(LINFO) << "Rescaling images";
    //Scale to size of RGB image but with keeping the ratio and data type
    double rowsScaleFactor = ((double)bigRgbImage.rows) / ((double)smallDispMap.rows);
    double colsScaleFactor = ((double)bigRgbImage.cols) / ((double)smallDispMap.cols);
    double scaleFactor = std::min(rowsScaleFactor, colsScaleFactor);

    cv::Mat dispMap(bigRgbImage.rows, bigRgbImage.cols, smallDispMap.type());

    cv::resize(smallDispMap, dispMap, cv::Size(0,0), scaleFactor, scaleFactor, cv::INTER_CUBIC);

    //Crop RGB image to disp map scale
    int x_margin = (bigRgbImage.cols - dispMap.cols) / 2;
    cv::Rect rgbCrop(x_margin, 0, dispMap.cols, dispMap.rows);
    cv::Mat rgbImage = bigRgbImage(rgbCrop);

    cv::Mat mask = cv::Mat::ones(rgbImage.rows, rgbImage.cols, CV_32FC1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(rgbCamInfo.width(), rgbCamInfo.height()));

    double fx_d =  0.001 / (irCamInfo.fx() * scaleFactor);
    double fy_d =  0.001 / (irCamInfo.fy() * scaleFactor);
    double cx_d = irCamInfo.cx() * scaleFactor;
    double cy_d = irCamInfo.cy() * scaleFactor;
    CLOG(LINFO) << "ir_cam_mat= " << irCamInfo.cameraMatrix();
    CLOG(LINFO) << "fx_d=" << fx_d << ", fy_d=" << fy_d << ", cx_d=" << cx_d << ",cy_d=" << cy_d;


    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin();
    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&dispMap.data[0]);

    int row_step = dispMap.step1();
    for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
        for (int u = 0; u < (int) cloud->width; ++u) {
            pcl::PointXYZRGB& pt = *pt_iter++;
            uint16_t depth = depth_row[u];

            // Missing points denoted by NaNs
            if (depth == 0 || mask.at<float>(v, u)==0) {
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }

            // Fill in XYZ
            pt.x = (u - cx_d) * depth * fx_d;
            pt.y = (v - cy_d) * depth * fy_d;
            pt.z = depth * 0.001;

            // Fill in RGB
            cv::Vec3b bgr = rgbImage.at<cv::Vec3b>(v, u);
            int b = bgr[0];
            int g = bgr[1];
            int r = bgr[2];
            //cout<< b << " " << g << " " << r << endl;// << " " << bgr[1] << " " << bgr[2]<<endl;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            /*pt.r = r;
            pt.g = g;
            pt.b = b;*/
        }
    }

    //TODO: zamienić true na parametr
    if(true){
        std::vector<int> indices;
        cloud->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }

    out_cloud_xyzrgb.write(cloud);
    out_rgb_image.write(rgbImage);
    out_disparity_image.write(dispMap);

}



} //: namespace Kinect2Device
} //: namespace Processors


