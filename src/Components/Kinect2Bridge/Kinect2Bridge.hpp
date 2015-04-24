/*!
 * \file Kinect2Bridge.hpp
 * \brief Ported from Kinect2Bridge driver for ROS
 * \author Dawid Kaczmarek
 */

#ifndef KINECT2BRIDGE_HPP_
#define KINECT2BRIDGE_HPP_

/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/CameraInfo.hpp"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>

#include <kinect2_definitions.h>
#include <depth_registration.h>

namespace Processors {
  namespace Kinect2Bridge {
    class Kinect2Bridge: public Base::Component {

    private:
      std::vector<int> compressionParams;
      std::string compression16BitExt, compression16BitString;

      cv::Size sizeColor, sizeIr, sizeLowRes;
      cv::Mat color, ir, depth;
      cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr;
      cv::Mat rotation, translation;
      cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;

      std::mutex lockIrDepth, lockColor;
      std::mutex lockSync, lockPub, lockTime;
      std::mutex lockRegLowRes, lockRegHighRes;

      Types::CameraInfo rgbCameraInfo, irCameraInfo;

      libfreenect2::Freenect2 freenect2;
      libfreenect2::Freenect2Device *device;
      libfreenect2::SyncMultiFrameListener *listenerColor, *listenerIrDepth;
      libfreenect2::PacketPipeline *packetPipeline;

      DepthRegistration *depthRegLowRes, *depthRegHighRes;

      size_t frameColor, frameIrDepth, pubFrameColor, pubFrameIrDepth;

      bool nextColor, nextIrDepth;
      double deltaT, depthShift, elapsedTimeColor, elapsedTimeIrDepth;
      bool running;

      // Output data streams
      Base::DataStreamOut<cv::Mat> out_rgb_image;
      Base::DataStreamOut<cv::Mat> out_ir_image;
      Base::DataStreamOut<cv::Mat> out_depth_map;
      Base::DataStreamOut<Types::CameraInfo> out_rgb_CameraInfo;
      Base::DataStreamOut<Types::CameraInfo> out_ir_CameraInfo;

      // Handlers
      Base::EventHandler2 h_getImages;

      // Properties
      Base::Property<double>    sensor;
      Base::Property<double>    fps_limit;
      Base::Property<string>    calib_path;
      Base::Property<bool>      use_png;
      Base::Property<int>       jpeg_quality;
      Base::Property<int>       png_level;
      Base::Property<string>    depth_method;
      Base::Property<int>       depth_device;
      Base::Property<string>    reg_method;
      Base::Property<int>       reg_device;
      Base::Property<double>    max_depth;
      Base::Property<double>    min_depth;
      Base::Property<bool>      bilateral_filter;
      Base::Property<bool>      edge_aware_filter;

      enum Image
      {
        IR = 0,
        IR_RECT,

        DEPTH,
        DEPTH_RECT,
        DEPTH_LORES,
        DEPTH_HIRES,

        COLOR,
        COLOR_RECT,
        COLOR_LORES,

        MONO,
        MONO_RECT,
        MONO_LORES,

        COUNT
      };

      enum Status
      {
        UNSUBCRIBED = 0,
        RAW,
        COMPRESSED,
        BOTH
      };


    public:
        /*!
         * Constructor.
         */
        Kinect2Bridge(const std::string & name = "Kinect2Bridge");

        /*!
         * Destructor
         */
        virtual ~Kinect2Bridge();

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

    private:

        bool initialize();

        bool initRegistration(const std::string &method, const int32_t device, const double maxDepth);

        bool initPipeline(const std::string &method, const int32_t device, const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth);

        bool initDevice(std::string &sensor);

        void initCalibration(const std::string &calib_path, const std::string &sensor);

        bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const;

        bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const;

        bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const;

        void createCameraInfo();

        void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, Types::CameraInfo &cameraInfo) const;

        void threadDispatcher();

        void receiveIrDepth();

        void receiveColor();

        void transferCameraInfo();

        bool receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames);

        void processColor(const cv::Mat &colorIn, cv::Mat &colorOut);

        void processIrDepth(const cv::Mat &ir, cv::Mat &irOut, const cv::Mat &depth, cv::Mat &depthOut);
    };
} //: namespace Kinect2Bridge
} //: namespace Processors

REGISTER_COMPONENT("Kinect2Bridge", Processors::Kinect2Bridge::Kinect2Bridge)

#endif /* KINECT2BRIDGE_HPP_ */
