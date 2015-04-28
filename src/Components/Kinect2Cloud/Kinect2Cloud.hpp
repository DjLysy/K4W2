/*!
 * \file
 * \brief 
 * \author Dawid Kaczmarek
 */

#ifndef KINECT2CLOUD_HPP_
#define KINECT2CLOUD_HPP_
#define LIBFREENECT2_THREADING_STDLIB

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/CameraInfo.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace Kinect2Cloud {

/*!
 * \class Kinect2Cloud
 * \brief Kinect2Cloud processor class.
 *
 * Captures RGB and depth data from Kinect for Windows V2 device. Based on libfreenect2 library
 */
class Kinect2Cloud: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Kinect2Cloud(const std::string & name = "Kinect2Cloud");

	/*!
	 * Destructor
	 */
	virtual ~Kinect2Cloud();

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
    Base::DataStreamIn<cv::Mat> in_rgb_image;
    Base::DataStreamIn<cv::Mat> in_disparity_image;
    Base::DataStreamIn<Types::CameraInfo> in_ir_camera_matrix;
    Base::DataStreamIn<Types::CameraInfo> in_rgb_camera_matrix;

	// Output data streams
    Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;
    Base::DataStreamOut<cv::Mat> out_rgb_image;
    Base::DataStreamOut<cv::Mat> out_disparity_image;


	// Handlers
    Base::EventHandler2 h_calculateCloud;

	// Properties
    //Base::Property<bool> enable_rgb;

    //Data Buffers
    cv::Mat color, depth;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    Types::CameraInfo rgbCamInfo;
    Types::CameraInfo irCamInfo;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	// Handlers
    void calculateCloud();

    // Methods
    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const;
    void createLookup(size_t width, size_t height);

};

} //: namespace Kinect2Device
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Kinect2Cloud", Processors::Kinect2Cloud::Kinect2Cloud)

#endif /* KINECT2CLOUD_HPP_ */
