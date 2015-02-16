/*!
 * \file
 * \brief
 * \author Dawid Kaczmarek
 */

#include <memory>
#include <string>

#include "Kinect2Device.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace Kinect2Device {

Kinect2Device::Kinect2Device(const std::string & name) :
		Base::Component(name) , 
		enable_rgb("enable_rgb", 1, "Boolean"), 
        enable_ir("enable_ir", 1, "Boolean"),
        enable_depth("enable_depth", 1, "Boolean")
{
	registerProperty(enable_rgb);
	registerProperty(enable_ir);
    registerProperty(enable_depth);

}

Kinect2Device::~Kinect2Device() {
    dev->stop();
    dev->close();
    delete freenect2;
    delete listener;
    delete frames;
}

void Kinect2Device::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_rgb_image", &out_rgb_image);
	registerStream("out_ir_image", &out_ir_image);
    registerStream("out_depth_map", &out_depth_map);
    registerStream("ir_cameraInfo", &out_ir_CameraInfo);
    registerStream("rgb_cameraInfo", &out_rgb_CameraInfo);
	// Register handlers
    //GetImages()
    h_getImages.setup(boost::bind(&Kinect2Device::getImages, this));
    registerHandler("getImages", &h_getImages);
    addDependency("getImages",NULL);
    //getCameraMatrices()
    h_getCameraMatrices.setup(boost::bind(&Kinect2Device::getCameraMatrices, this));
    registerHandler("getCameraMatrices", &h_getCameraMatrices);
    addDependency("getCameraMatrices",NULL);
}

bool Kinect2Device::onInit() {
    if(dev == 0)
    {
      std::cout << "no device connected or failure opening the default one!" << std::endl;
      return false;
    }
    return true;
}

bool Kinect2Device::onFinish() {
	return true;
}

bool Kinect2Device::onStop() {
	return true;
}

bool Kinect2Device::onStart() {
    freenect2 = new libfreenect2::Freenect2();
    try {
    dev = freenect2->openDefaultDevice();
    } catch (...)
    {
        return false;
    }
    sleep(2);

    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    frames = new libfreenect2::FrameMap();

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();
    sleep(5);
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	return true;
}

void Kinect2Device::getCameraMatrices() {
    cv::Mat cameraMatrixColor, distortionColor;
    cv::Mat cameraMatrixIr, distortionIr;
    cv::Mat rotation, translation;

    Types::CameraInfo rgb_camera_info, ir_camera_info;

    libfreenect2::Freenect2Device::ColorCameraParams colorParams = dev->getColorCameraParams();
    libfreenect2::Freenect2Device::IrCameraParams    irParams    = dev->getIrCameraParams();

    cameraMatrixColor   = cv::Mat::eye(3, 3, CV_32FC1);
    distortionColor     = cv::Mat::zeros(1, 5, CV_32FC1);
    cameraMatrixIr      = cv::Mat::eye(3, 3, CV_32FC1);
    distortionIr        = cv::Mat::zeros(1, 5, CV_32FC1);

    cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
    cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
    cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
    cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
    cameraMatrixColor.at<double>(2, 2) = 1;

    cameraMatrixIr.at<double>(0, 0) = irParams.fx;
    cameraMatrixIr.at<double>(1, 1) = irParams.fy;
    cameraMatrixIr.at<double>(0, 2) = irParams.cx;
    cameraMatrixIr.at<double>(1, 2) = irParams.cy;
    cameraMatrixIr.at<double>(2, 2) = 1;

    distortionIr.at<double>(0, 0) = irParams.k1;
    distortionIr.at<double>(0, 1) = irParams.k2;
    distortionIr.at<double>(0, 2) = irParams.p1;
    distortionIr.at<double>(0, 3) = irParams.p2;

    rotation = cv::Mat::eye(3, 3, CV_64F);
    translation = cv::Mat::zeros(3, 1, CV_64F);

    rgb_camera_info.setCameraMatrix(cameraMatrixColor);
    rgb_camera_info.setDistCoeffs(distortionColor);

    ir_camera_info.setCameraMatrix(cameraMatrixIr);
    ir_camera_info.setDistCoeffs(distortionIr);

    out_ir_CameraInfo.write(ir_camera_info);
    out_rgb_CameraInfo.write(rgb_camera_info);
}

void Kinect2Device::getImages() {
    listener->waitForNewFrame(*frames);

    libfreenect2::Frame *rgb = (*frames)[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = (*frames)[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = (*frames)[libfreenect2::Frame::Depth];

    cv::Mat rgbImg(rgb->height, rgb->width, CV_8UC3, rgb->data);
    cv::Mat irImg(ir->height, ir->width, CV_32FC1, ir->data);
    cv::Mat depthImg(depth->height, depth->width, CV_32FC1, depth->data);

    //Copy buffer content to local copies of data to provide asynchronous access to snapshot of data
    rgbImg.copyTo(imgBuffer);
    irImg.convertTo(irBuffer, CV_8UC3, 255.0f / 65535.0f );
    depthImg.convertTo(depthBuffer, CV_8UC3, 255.0f / 4500.0f );

    listener->release(*frames);

    out_depth_map.write(imgBuffer);
    out_rgb_image.write(depthBuffer);
    out_ir_image.write(irBuffer);

    this->getCameraMatrices();
}



} //: namespace Kinect2Device
} //: namespace Processors


