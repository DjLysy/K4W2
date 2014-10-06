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
        enable_depth("enable_depth", 1, "Boolean"),
        listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
{
	registerProperty(enable_rgb);
	registerProperty(enable_ir);
	registerProperty(enable_depth);

}

Kinect2Device::~Kinect2Device() {
}

void Kinect2Device::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_rgb_image", &out_rgb_image);
	registerStream("out_ir_image", &out_ir_image);
	registerStream("out_depth_map", &out_depth_map);
	// Register handlers
	h_getIRImage.setup(boost::bind(&Kinect2Device::getIRImage, this));
	registerHandler("getIRImage", &h_getIRImage);
	h_getRGBImage.setup(boost::bind(&Kinect2Device::getRGBImage, this));
	registerHandler("getRGBImage", &h_getRGBImage);
	h_getDepthMap.setup(boost::bind(&Kinect2Device::getDepthMap, this));
	registerHandler("getDepthMap", &h_getDepthMap);

}

bool Kinect2Device::onInit() {
    libfreenect2::Freenect2 freenect2;
    dev = freenect2.openDefaultDevice();
    if(dev == 0)
    {
      std::cout << "no device connected or failure opening the default one!" << std::endl;
      return false;
    }
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	return true;
}

bool Kinect2Device::onFinish() {
    dev->close();
	return true;
}

bool Kinect2Device::onStop() {
    dev->stop();
	return true;
}

bool Kinect2Device::onStart() {
    dev->start();
    listener.waitForNewFrame(frames);
	return true;
}

void Kinect2Device::getIRImage() {
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    out_ir_image.write(cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
}

void Kinect2Device::getRGBImage() {
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    out_rgb_image.write(cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
}

void Kinect2Device::getDepthMap() {
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    out_depth_map.write(cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);
}



} //: namespace Kinect2Device
} //: namespace Processors


