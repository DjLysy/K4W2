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

    freenect2 = new libfreenect2::Freenect2();
    dev = freenect2->openDefaultDevice();

    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    frames = new libfreenect2::FrameMap();

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
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
	// Register handlers
    h_getImages.setup(boost::bind(&Kinect2Device::getImages, this));
    registerHandler("getImages", &h_getImages);
    addDependency("getImages",NULL);
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
	return true;
}

void Kinect2Device::getImages() {
    listener->waitForNewFrame(*frames);

    libfreenect2::Frame *rgb = (*frames)[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = (*frames)[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = (*frames)[libfreenect2::Frame::Depth];

    cv::Mat rgbImg(rgb->height, rgb->width, CV_8UC3, rgb->data);
    cv::Mat irImg(ir->height, ir->width, CV_32FC1, ir->data);
    cv::Mat depthImg(depth->height, depth->width, CV_32FC1, depth->data);
    irImg = irImg / 20000.0f;
    depthImg = depthImg / 4500.0f;

    //Copy buffer content to local copies of data to provide asynchronous access to snapshot of data
    rgbImg.copyTo(imgBuffer);
    depthImg.copyTo(depthBuffer);
    irImg.copyTo(irBuffer);

    out_depth_map.write(imgBuffer);
    out_rgb_image.write(depthBuffer);
    out_ir_image.write(irBuffer);

    listener->release(*frames);
}



} //: namespace Kinect2Device
} //: namespace Processors


