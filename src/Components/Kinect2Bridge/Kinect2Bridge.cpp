#include "Kinect2Bridge.hpp"

namespace Processors {
namespace Kinect2Bridge {

Kinect2Bridge::Kinect2Bridge(const std::string & name)
  : Base::Component(name),
    sensor("sensor",double(-1.0)),
    fps_limit("fps_limit",double(-1.0)),
    calib_path("calib_path", std::string(".//")),
    use_png("use_png",false),
    jpeg_quality("jpeg_quality",int(90)),
    png_level("png_level",int(1)),
    depth_method("depth_method",std::string("cpu")),
    depth_device("depth_device",int(1)),
    reg_method("reg_method",std::string("default")),
    reg_device("reg_device",int(-1)),
    max_depth("max_depth",double(12.0)),
    min_depth("min_depth",double(0.1)),
    bilateral_filter("bilateral_filter",true),
    edge_aware_filter("edge_aware_filter", true)
{
    registerProperty(sensor);
    registerProperty(fps_limit);
    registerProperty(calib_path);
    registerProperty(use_png);
    registerProperty(jpeg_quality);
    registerProperty(png_level);
    registerProperty(depth_method);
    registerProperty(depth_device);
    registerProperty(reg_method);
    registerProperty(reg_device);
    registerProperty(max_depth);
    registerProperty(min_depth);
    registerProperty(bilateral_filter);
    registerProperty(edge_aware_filter);

    sizeColor = cv::Size(1920, 1080);
    sizeIr = cv::Size(512, 424);
    sizeLowRes = cv::Size(sizeColor.width / 2, sizeColor.height / 2);
    frameColor = 0;
    frameIrDepth = 0;
    pubFrameColor = 0;
    pubFrameIrDepth = 0;
    nextColor = false;
    nextIrDepth = false;
    depthShift = 0;
    running = false;
}


Kinect2Bridge::~Kinect2Bridge()
{
    delete listenerIrDepth;
    delete listenerColor;
}

void Kinect2Bridge::prepareInterface() {
    registerStream("out_rgb_image", &out_rgb_image);
    registerStream("out_ir_image", &out_ir_image);
    registerStream("out_depth_map", &out_depth_map);
    registerStream("out_rgb_CameraInfo", &out_rgb_CameraInfo);
    registerStream("out_ir_CameraInfo", &out_ir_CameraInfo);

    // Register handlers
    h_getImages.setup(boost::bind(&Kinect2Bridge::threadDispatcher, this));
    registerHandler("getImages", &h_getImages);
    addDependency("getImages",NULL);
}


bool Kinect2Bridge::onInit() {
    running = initialize();
    return running;
}

bool Kinect2Bridge::onFinish() {

    running = false;
    device->stop();
    device->close();
    return true;
}

bool Kinect2Bridge::onStop() {

    return true;
}

bool Kinect2Bridge::onStart() {

    return true;
}


bool Kinect2Bridge::initialize()
{
  std::string sensor;

  double fps_limit = this->fps_limit;
  std::string calib_path = this->calib_path;
  std::string depthDefault = "cpu";
  std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
  depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
  depthDefault = "opencl";
#endif
#ifdef DEPTH_REG_OPENCL
  regDefault = "opencl";
#endif

  this->depth_method = depthDefault;
  this->reg_method = regDefault;

  if(this->sensor > 0)
  {
    sensor = std::to_string((uint64_t)this->sensor);
  }

  deltaT = fps_limit > 0 ? 1.0 / fps_limit : 0.0;

  if(calib_path.empty() || calib_path.back() != '/')
  {
    calib_path += '/';
  }

  bool ret = true;
  ret = ret && initPipeline(this->depth_method, this->depth_device, this->bilateral_filter, this->edge_aware_filter, this->min_depth, this->max_depth);
  ret = ret && initDevice(sensor);

  if(ret)
  {
    initCalibration(this->calib_path, sensor);
  }

  ret = ret && initRegistration(this->reg_method, this->reg_device, this->max_depth);

  if(ret)
  {
    createCameraInfo();
  }

  return ret;
}

bool Kinect2Bridge::initRegistration(const std::string &method, const int32_t device, const double maxDepth)
{
  DepthRegistration::Method reg;

  if(method == "default")
  {
    reg = DepthRegistration::DEFAULT;
  }
  else if(method == "cpu")
  {
#ifdef DEPTH_REG_CPU
    reg = DepthRegistration::CPU;
#else
    std::cerr << "CPU registration is not available!" << std::endl;
    return -1;
#endif
  }
  else if(method == "opencl")
  {
#ifdef DEPTH_REG_OPENCL
    reg = DepthRegistration::OPENCL;
#else
    std::cerr << "OpenCL registration is not available!" << std::endl;
    return -1;
#endif
  }
  else
  {
    std::cerr << "Unknown registration method: " << method << std::endl;
    return false;
  }

  depthRegLowRes = DepthRegistration::New(reg);
  depthRegHighRes = DepthRegistration::New(reg);

  bool ret = true;
  ret = ret && depthRegLowRes->init(cameraMatrixLowRes, sizeLowRes, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, device);
  ret = ret && depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, device);

  return ret;
}

bool Kinect2Bridge::initPipeline(const std::string &method, const int32_t device, const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
{
  if(method == "default")
  {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
    packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
    packetPipeline = new libfreenect2::CpuPacketPipeline();
#endif
  }
  else if(method == "cpu")
  {
    packetPipeline = new libfreenect2::CpuPacketPipeline();
  }
  else if(method == "opencl")
  {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#else
    std::cerr << "OpenCL depth processing is not available!" << std::endl;
    return false;
#endif
  }
  else if(method == "opengl")
  {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
    std::cerr << "OpenGL depth processing is not available!" << std::endl;
    return false;
#endif
  }
  else
  {
    std::cerr << "Unknown depth processing method: " << method << std::endl;
    return false;
  }

  libfreenect2::DepthPacketProcessor::Config config;
  config.EnableBilateralFilter = bilateral_filter;
  config.EnableEdgeAwareFilter = edge_aware_filter;
  config.MinDepth = minDepth;
  config.MaxDepth = maxDepth;
  packetPipeline->getDepthPacketProcessor()->setConfiguration(config);
  return true;
}

bool Kinect2Bridge::initDevice(std::string &sensor)
{
  bool deviceFound = false;
  const int numOfDevs = freenect2.enumerateDevices();

  if(numOfDevs <= 0)
  {
    std::cerr << "Error: no Kinect2 devices found!" << std::endl;
    return false;
  }

  if(sensor.empty())
  {
    sensor = freenect2.getDefaultDeviceSerialNumber();
  }

  std::cout << "Kinect2 devices found: " << std::endl;
  for(int i = 0; i < numOfDevs; ++i)
  {
    const std::string &s = freenect2.getDeviceSerialNumber(i);
    deviceFound = deviceFound || s == sensor;
    std::cout << "  " << i << ": " << s << (s == sensor ? " (selected)" : "") << std::endl;
  }

  if(!deviceFound)
  {
    std::cerr << "Error: Device with serial '" << sensor << "' not found!" << std::endl;
    return false;
  }

  device = freenect2.openDevice(sensor, packetPipeline);

  if(device == 0)
  {
    std::cout << "no device connected or failure opening the default one!" << std::endl;
    return -1;
  }

  listenerColor = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
  listenerIrDepth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

  device->setColorFrameListener(listenerColor);
  device->setIrAndDepthFrameListener(listenerIrDepth);

  std::cout << std::endl << "starting kinect2" << std::endl << std::endl;
  device->start();

  std::cout << std::endl << "device serial: " << sensor << std::endl;
  std::cout << "device firmware: " << device->getFirmwareVersion() << std::endl;

  libfreenect2::Freenect2Device::ColorCameraParams colorParams = device->getColorCameraParams();
  libfreenect2::Freenect2Device::IrCameraParams irParams = device->getIrCameraParams();

  std::cout << std::endl << "default ir camera parameters: " << std::endl;
  std::cout << "fx " << irParams.fx << ", fy " << irParams.fy << ", cx " << irParams.cx << ", cy " << irParams.cy << std::endl;
  std::cout << "k1 " << irParams.k1 << ", k2 " << irParams.k2 << ", p1 " << irParams.p1 << ", p2 " << irParams.p2 << ", k3 " << irParams.k3 << std::endl;

  std::cout << std::endl << "default color camera parameters: " << std::endl;
  std::cout << "fx " << colorParams.fx << ", fy " << colorParams.fy << ", cx " << colorParams.cx << ", cy " << colorParams.cy << std::endl;

  cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
  distortionColor = cv::Mat::zeros(1, 5, CV_64F);

  cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
  cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
  cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
  cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
  cameraMatrixColor.at<double>(2, 2) = 1;

  cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
  distortionIr = cv::Mat::zeros(1, 5, CV_64F);

  cameraMatrixIr.at<double>(0, 0) = irParams.fx;
  cameraMatrixIr.at<double>(1, 1) = irParams.fy;
  cameraMatrixIr.at<double>(0, 2) = irParams.cx;
  cameraMatrixIr.at<double>(1, 2) = irParams.cy;
  cameraMatrixIr.at<double>(2, 2) = 1;

  distortionIr.at<double>(0, 0) = irParams.k1;
  distortionIr.at<double>(0, 1) = irParams.k2;
  distortionIr.at<double>(0, 2) = irParams.p1;
  distortionIr.at<double>(0, 3) = irParams.p2;
  distortionIr.at<double>(0, 4) = irParams.k3;

  rotation = cv::Mat::eye(3, 3, CV_64F);
  translation = cv::Mat::zeros(3, 1, CV_64F);
  translation.at<double>(0) = -0.0520;
  return true;
}

void Kinect2Bridge::initCalibration(const std::string &calib_path, const std::string &sensor)
{
  std::string calibPath = calib_path + sensor + '/';

  struct stat fileStat;
  bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
  if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
  {
    std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
  }

  if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixIr, distortionIr))
  {
    std::cerr << "using sensor defaults for ir intrinsic parameters." << std::endl;
  }

  if(calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
  {
    std::cerr << "using defaults for rotation and translation." << std::endl;
  }

  if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
  {
    std::cerr << "using defaults for depth shift." << std::endl;
    depthShift = 0.0;
  }

  cameraMatrixLowRes = cameraMatrixColor.clone();
  cameraMatrixLowRes.at<double>(0, 0) /= 2;
  cameraMatrixLowRes.at<double>(1, 1) /= 2;
  cameraMatrixLowRes.at<double>(0, 2) /= 2;
  cameraMatrixLowRes.at<double>(1, 2) /= 2;

  const int mapType = CV_16SC2;
  cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
  cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
  cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);

  std::cout << std::endl << "camera parameters used:" << std::endl
            << "camera matrix color:" << std::endl << cameraMatrixColor << std::endl
            << "distortion coefficients color:" << std::endl << distortionColor << std::endl
            << "camera matrix ir:" << std::endl << cameraMatrixIr << std::endl
            << "distortion coefficients ir:" << std::endl << distortionIr << std::endl
            << "rotation:" << std::endl << rotation << std::endl
            << "translation:" << std::endl << translation << std::endl
            << "depth shift:" << std::endl << depthShift << std::endl << std::endl;
}

bool Kinect2Bridge::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
    fs[K2_CALIB_DISTORTION] >> distortion;
    fs.release();
  }
  else
  {
    std::cerr << "can't open calibration file: " << filename << std::endl;
    return false;
  }
  return true;
}

bool Kinect2Bridge::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_ROTATION] >> rotation;
    fs[K2_CALIB_TRANSLATION] >> translation;
    fs.release();
  }
  else
  {
    std::cerr << "can't open calibration pose file: " << filename << std::endl;
    return false;
  }
  return true;
}

bool Kinect2Bridge::loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
    fs.release();
  }
  else
  {
    std::cerr << "can't open calibration depth file: " << filename << std::endl;
    return false;
  }
  return true;
}

void Kinect2Bridge::createCameraInfo()
{
  cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F);
  cv::Mat projIr = cv::Mat::zeros(3, 4, CV_64F);

  cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
  cameraMatrixIr.copyTo(projIr(cv::Rect(0, 0, 3, 3)));

  createCameraInfo(sizeColor, cameraMatrixColor, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projColor, rgbCameraInfo);
  createCameraInfo(sizeIr, cameraMatrixIr, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projIr, irCameraInfo);
}

void Kinect2Bridge::createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, Types::CameraInfo &cameraInfo) const
{
    cameraInfo.setCameraMatrix(cameraMatrix);
    cameraInfo.setProjectionMatrix(projection);
    cameraInfo.setDistCoeffs(distortion);
    cameraInfo.setRotationMatrix(rotation);
    cameraInfo.setSize(size);
}

void Kinect2Bridge::threadDispatcher()
{
    receiveIrDepth();
    receiveColor();
    transferCameraInfo();
}

void Kinect2Bridge::receiveIrDepth()
{
  libfreenect2::FrameMap frames;
  libfreenect2::Frame *irFrame, *depthFrame;
  cv::Mat depth, depthOut, ir, irOut;
  size_t frame;

  if(!receiveFrames(listenerIrDepth, frames))
  {
    return;
  }
  //double now = ros::Time::now().toSec();

  irFrame = frames[libfreenect2::Frame::Ir];
  depthFrame = frames[libfreenect2::Frame::Depth];

  ir = cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data);
  depth = cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data);

  frame = frameIrDepth++;
  //lockIrDepth.unlock();

  processIrDepth(ir, irOut, depth, depthOut);
  listenerIrDepth->release(frames);


  //drukowanie do streamów
  out_ir_image.write(irOut);
  out_depth_map.write(depthOut);

  //double elapsed = ros::Time::now().toSec() - now;
  //lockTime.lock();
  //elapsedTimeIrDepth += elapsed;
  //lockTime.unlock();
}

void Kinect2Bridge::receiveColor()
{
  libfreenect2::FrameMap frames;
  libfreenect2::Frame *colorFrame;
  cv::Mat color, colorOut;
  size_t frame;

  if(!receiveFrames(listenerColor, frames))
  {
    return;
  }
  //double now = ros::Time::now().toSec();

  //header = createHeader(lastColor, lastDepth);

  colorFrame = frames[libfreenect2::Frame::Color];
  color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC3, colorFrame->data);

  frame = frameColor++;
  //lockColor.unlock();

  processColor(color, colorOut);

  listenerColor->release(frames);

  //drukowanie do streamów
  out_rgb_image.write(colorOut);

  //double elapsed = ros::Time::now().toSec() - now;
  //lockTime.lock();
  //elapsedTimeColor += elapsed;
  //lockTime.unlock();
}

void Kinect2Bridge::transferCameraInfo()
{
    out_ir_CameraInfo.write(irCameraInfo);
    out_rgb_CameraInfo.write(rgbCameraInfo);
}

bool Kinect2Bridge::receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames)
{
  bool newFrames = false;
  for(; !newFrames;)
  {
#ifdef LIBFREENECT2_THREADING_STDLIB
    newFrames = listener->waitForNewFrame(frames, 1000);
#else
    newFrames = true;
    listener->waitForNewFrame(frames);
#endif
    if(!running)
    {
      if(newFrames)
      {
        listener->release(frames);
      }
      return false;
    }
  }
  return true;
}

void Kinect2Bridge::processColor(const cv::Mat &colorIn, cv::Mat &colorOut)
{
    //COLOR
    cv::Mat tempColor;
    cv::flip(colorIn, tempColor, 1);
    cv::remap(tempColor, colorOut, map1Color, map2Color, cv::INTER_AREA);
}

void Kinect2Bridge::processIrDepth(const cv::Mat &ir, cv::Mat &irOut, const cv::Mat &depth, cv::Mat &depthOut)
{
    //IR
    cv::Mat tempIr, tempDepth;
    ir.convertTo(tempIr, CV_16U);
    cv::flip(tempIr, tempIr, 1);
    cv::remap(tempIr, irOut, map1Ir, map2Ir, cv::INTER_AREA);

    //DEPTH
    depth.convertTo(tempDepth, CV_16U, 1, depthShift);
    cv::flip(tempDepth, tempDepth, 1);
    depthRegHighRes->registerDepth(tempDepth, depthOut);
    //cv::remap(tempDepth, depthOut, map1Ir, map2Ir, cv::INTER_NEAREST);

}

} //: namespace Kinect2Bridge
} //: namespace Processors
