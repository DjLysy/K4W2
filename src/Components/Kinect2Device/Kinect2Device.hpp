/*!
 * \file
 * \brief 
 * \author Dawid Kaczmarek
 */

#ifndef KINECT2DEVICE_HPP_
#define KINECT2DEVICE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>


namespace Processors {
namespace Kinect2Device {

/*!
 * \class Kinect2Device
 * \brief Kinect2Device processor class.
 *
 * Captures RGB and depth data from Kinect for Windows V2 device. Based on libfreenect2 library
 */
class Kinect2Device: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Kinect2Device(const std::string & name = "Kinect2Device");

	/*!
	 * Destructor
	 */
	virtual ~Kinect2Device();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams

	// Output data streams
    Base::DataStreamOut<cv::Mat> out_rgb_image;
    Base::DataStreamOut<cv::Mat> out_ir_image;
    Base::DataStreamOut<cv::Mat> out_depth_map;

	// Handlers
	Base::EventHandler2 h_getIRImage;
	Base::EventHandler2 h_getRGBImage;
	Base::EventHandler2 h_getDepthMap;

	// Properties
	Base::Property<Boolean> enable_rgb;
	Base::Property<Boolean> enable_ir;
	Base::Property<Boolean> enable_depth;

    // Others

    libfreenect2::Freenect2Device *dev;
    libfreenect2::SyncMultiFrameListener listener;
    libfreenect2::FrameMap frames;

	// Handlers
	void getIRImage();
	void getRGBImage();
	void getDepthMap();

};

} //: namespace Kinect2Device
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Kinect2Device", Processors::Kinect2Device::Kinect2Device)

#endif /* KINECT2DEVICE_HPP_ */
