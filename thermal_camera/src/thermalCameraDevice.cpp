#include "thermalCameraDevice.hpp"

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

#define OPENCV_DEBUG 0
#define IRLIB_DEBUG 0

#define POOLING_TIMEOUT 40 // MS, DON'T CHANGE, the min period of camera is 1000/32 = 31.25

thermalCameraDevice::thermalCameraDevice()
{
  // Workaround to link callbacks to class member functions
  // that are non-static, a better solution is to pass the
  // pointer through the "void* arg" parameter
  thisPtr_ = this;
}

thermalCameraDevice::~thermalCameraDevice()
{
  thisPtr_ = NULL;
}


void thermalCameraDevice::clearqThermal(std::stack<Image<unsigned short>*> &q)
{
  while (!q.empty())
  {
    delete q.top();
    q.pop();
  }
}

void thermalCameraDevice::clearqFloat(std::stack<Image<float>*> &q)
{
  while (!q.empty())
  {
    delete q.top();
    q.pop();
  }
}


long long thermalCameraDevice::obtainRGBThermalImage(unsigned char* &data)
{
  long long timestamp = -1;
  int wInit = imager_->getWidth();
  int hInit = imager_->getHeight();
  int image_size = wInit * hInit * 3;
  
  if (!qThermal_.empty())
  {  
    pthread_mutex_lock(&thermal_mutex_);

    Image<unsigned short>* img = qThermal_.top();
    
    pthread_mutex_unlock(&thermal_mutex_);

    unsigned char* bufferThermal = new unsigned char[image_size];

    // IRDeviceParams::IRFrameMetaDat.timestamp is in 1e7/second.
    // Multiply by 100 to convert to nanoseconds.
    timestamp = img->_timestamp * 100;

    iBuilder_.setData(img->_width, img->_height, img->_data);
    wInit = iBuilder_.getStride();
    hInit = img->_height;
    iBuilder_.convertTemperatureToPaletteImage(bufferThermal);
      
    // Data copied to return
    std::memcpy(data, bufferThermal, image_size);

    if(OPENCV_DEBUG)
    {
      cv::Mat img_cv = cv::Mat(hInit, wInit, CV_8UC3, bufferThermal);
      cv::cvtColor(img_cv, img_cv, cv::COLOR_BGR2RGB);
      cv::namedWindow("Thermal RGB Image", cv::WINDOW_AUTOSIZE );
      cv::imshow("Thermal RGB Image", img_cv);
      cv::waitKey(1);
    }

    pthread_mutex_lock(&thermal_mutex_);
    clearqThermal(qThermal_);
    pthread_mutex_unlock(&thermal_mutex_);

    if(bufferThermal) delete [] bufferThermal;
  }

  return timestamp;
}

long long thermalCameraDevice::obtainFloatThermalImage(float* &data)
{
  long long timestamp = -1;
  int wInit = imager_->getWidth();
  int hInit = imager_->getHeight();
  int image_size = wInit * hInit * 4;
  
  if (!qFloat_.empty())
  {  
    pthread_mutex_lock(&float_mutex_);
    
    Image<float>* img = qFloat_.top();
    
    pthread_mutex_unlock(&float_mutex_);

    float* bufferFloat       =  new float[image_size];
    float* bufferNormalized  =  new float[image_size];
    timestamp = img->_timestamp * 100; // convert to nanoseconds

    for (int i = 0; i < image_size; i++)
    {
      bufferFloat[i]       =  img->_data[i];
      bufferNormalized[i]  = (img->_data[i] - params_.tMin) / (params_.tMax - params_.tMin);
    }
    
    // Data copied to return
    std::memcpy(data, bufferFloat, image_size);
    
    if(OPENCV_DEBUG)
    {
      cv::Mat img_cv = cv::Mat(hInit, wInit, CV_32FC1, bufferNormalized);
      cv::namedWindow("Thermal Float Image", cv::WINDOW_AUTOSIZE);
      cv::imshow("Thermal Float Image", img_cv);
      cv::waitKey(1);
    }

    pthread_mutex_lock(&float_mutex_);
    clearqFloat(qFloat_);
    pthread_mutex_unlock(&float_mutex_);

    if(bufferFloat) delete [] bufferFloat;
    if(bufferNormalized) delete [] bufferNormalized;
  }

  return timestamp;
}

bool thermalCameraDevice::configureCamera(std::string config_xml)
{
  if(IRLIB_DEBUG)
  {
    evo::IRLogger::setVerbosity(evo::IRLOG_DEBUG, evo::IRLOG_DEBUG);
  }
  else
  {
    evo::IRLogger::setVerbosity(evo::IRLOG_ERROR, evo::IRLOG_OFF);
  }
  

  // Read parameters from xml file
  if(!evo::IRDeviceParamsReader::readXML(config_xml.c_str(), params_))
  {      
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "XML configuration file cannot be read");
    return false;
  }

  // Find valid device
  dev_ = evo::IRDevice::IRCreateDevice(params_);
  if(!dev_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error: Device with serial# %ld", params_.serial);

    return false;
  }

  // Specify calibration
  evo::IRCalibrationManager caliManager(params_.caliPath, params_.formatsPath);
  
  // Output available optics
  const evo::IRArray<evo::IROptics> *optics = caliManager.getAvailableOptics(dev_->getSerial());
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "Available optics for camera with serial: %ld", dev_->getSerial());

  for (unsigned int i = 0; i < optics->size(); i++)
  {
    evo::IROptics op = (*optics)[i];
    RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "FOV: %d  deg, Text: %s", op.fov, op.text.data());

    for (unsigned int j = 0; j < op.tempRanges.size(); j++)
    {
      RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "tMin: %d C, tMax: %d C" , op.tempRanges[j].tMin, op.tempRanges[j].tMax);
    }  
  }

  // Initialize Optris image processing chain
  imager_ = new evo::IRImager();
  bool init_result = imager_->init(&params_, dev_->getFrequency(), dev_->getWidth(), dev_->getHeight(), dev_->controlledViaHID(),dev_->getHwRev(), dev_->getFwRev());

  if(init_result)
  {
    if(imager_->getWidth()==0 || imager_->getHeight()==0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error: Image streams not available or wrongly configured. Check connection of camera and config file.");
      return false;
    }

    // Set radiation parameters of thermal image
    imager_->setRadiationParameters(1.0, 1.0);
    imager_->setTempRange(params_.tMin, params_.tMax);

    // Updating public variables
    thermal_min_ = params_.tMin;
    thermal_min_ = params_.tMax;
    thermal_width_  = imager_->getWidth();
    thermal_height_ = imager_->getHeight();

    // --- Set callback methods and start video streaming ----
    dev_->setRawFrameCallback(onRawFrame_wrapper);
    imager_->setThermalFrameCallback(onThermalFrame_wrapper);
    imager_->setVisibleFrameCallback(onVisibleFrame_wrapper);
    imager_->setFlagStateCallback(onFlageStateChange_wrapper);
    imager_->setThermalFrameEventCallback(onThermalFrameEvent_wrapper);
    imager_->setVisibleFrameEventCallback(onVisibleFrameEvent_wrapper);
    imager_->setProcessExitCallback(onProcessExit_wrapper);

    iBuilder_ = evo::ImageBuilder(true, imager_->getTemprangeDecimal());
    if(dev_->startStreaming()!=0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error occurred in starting stream ... aborting. You may need to reconnect the camera.");
      return false;
    }

    // First pooling job to init the camera
    double timestamp;
    int i = 0;
    while(i < 5)
    {
      unsigned char* bufferRaw = new unsigned char[dev_->getRawBufferSize()];
      dev_->getFrame(bufferRaw, &timestamp);
      i++;

      if(bufferRaw) delete [] bufferRaw;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error while device init.");
    return false;
  }

  return true;
}


evo::IRDeviceError thermalCameraDevice::processCameraData()
{
  double timestamp = -1;
  unsigned char* bufferRaw = NULL;
  evo::IRDeviceError retval;

  switch(irState_)
  {
    case IRIMAGER_STATE_ACQUIRE:
      
      bufferRaw = new unsigned char[dev_->getRawBufferSize()];
      retval = dev_->getFrame(bufferRaw, &timestamp, POOLING_TIMEOUT);
      delete [] bufferRaw;

      if(retval == evo::IRIMAGER_SUCCESS)
      {
        RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "IRIMAGER_SUCCESS state entered");
      }
      else if(retval == evo::IRIMAGER_NOSYNC)
      {
        RCLCPP_WARN(rclcpp::get_logger(__PRETTY_FUNCTION__), "IRIMAGER_NOSYNC state entered");
      }
      else if(retval == evo::IRIMAGER_DISCONNECTED)
      {
        delete dev_;
        dev_ = NULL;
        irState_ = IRIMAGER_STATE_ROAMING;
        RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "IRIMAGER_DISCONNECTED state entered");
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger(__PRETTY_FUNCTION__), "WARNING: Imager returned error code %d", retval);
      }

    break;
    
    case IRIMAGER_STATE_ROAMING:
      
      RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Imager was disconnected ... trying to recover connection state");

      unsigned long serial = 0;
      evo::IRCalibrationManager::findSerial(serial);
      
      if(serial==0)
      {
        RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "NO DEVICE FOUND!");
      }
      else
      {
        if(params_.serial != serial)
        {
          RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Imager hardware was changed ... skipping");
        }

        if(dev_==NULL)
        {
          dev_ = evo::IRDevice::IRCreateDevice(params_);
        }

        if(dev_!=NULL)
        {
          imager_->reconnect(&params_, dev_->getFrequency(), dev_->getWidth(), dev_->getHeight(), 
                          dev_->controlledViaHID(), dev_->getHwRev(), dev_->getFwRev());
          if(dev_->isOpen())
          {
            dev_->setRawFrameCallback(onRawFrame_wrapper);
            imager_->setThermalFrameCallback(onThermalFrame_wrapper);
            imager_->setVisibleFrameCallback(onVisibleFrame_wrapper);
            imager_->setFlagStateCallback(onFlageStateChange_wrapper);
            imager_->setThermalFrameEventCallback(onThermalFrameEvent_wrapper);
            imager_->setVisibleFrameEventCallback(onVisibleFrameEvent_wrapper);
            dev_->startStreaming();
            irState_ = IRIMAGER_STATE_ACQUIRE;
          }
        }
      }
    break;
  }

  std::cout << std::flush;
  return retval;
}

bool thermalCameraDevice::stopCamera()
{
  dev_->stopStreaming();
  
  delete imager_;
  delete dev_;

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Stopping camera");

  return true;
}

void thermalCameraDevice::onThermalFrame(unsigned short* thermal, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
{
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "onThermalFrame callback");

  if(showVisibleChannel_ || !imager_->isFlagOpen()) return;

  pthread_mutex_lock(&thermal_mutex_);
  imgThermal_ = new Image<unsigned short>(w, h, meta.timestamp, thermal);
  pthread_mutex_unlock(&thermal_mutex_);

  // Creating image of floats
  // http://documentation.evocortex.com/libirimager2/html/index.html
  float* float_buff =  new float[w*h];
  short decimalPlaces = imager_->getTemprangeDecimal();
  float divisor = std::pow(10, decimalPlaces);
  for (unsigned int i=0 ; i < w*h; i++)
    float_buff[i] = (float)thermal[i] / divisor - 100.f;

  pthread_mutex_lock(&float_mutex_);
  imgFloat_ = new Image<float>(w, h,  meta.timestamp, float_buff);
  delete [] float_buff;
  pthread_mutex_unlock(&float_mutex_);
}


void thermalCameraDevice::onVisibleFrame(unsigned char* yuyv, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
{
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "onVisibleFrame callback");

  if(!showVisibleChannel_) return;
  pthread_mutex_lock(&yuyv_mutex_);
  imgYUYV_ = new Image<unsigned char>(w, h,  meta.timestamp, yuyv);
  pthread_mutex_unlock(&yuyv_mutex_);
}

void thermalCameraDevice::onFlageStateChange(evo::EnumFlagState fs, void* arg)
{
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "onFlageStateChange callback with flag state: %d", fs);
}

void thermalCameraDevice::onThermalFrameEvent(unsigned short* thermal, unsigned short* energy, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, const evo::IRArray<evo::IREventData>& events, void* arg)
{
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "onThermalFrameEvent callback");

  if(showVisibleChannel_ || !imager_->isFlagOpen()) return;
  pthread_mutex_lock(&thermal_mutex_);
  imgThermal_ = new Image<unsigned short>(w, h,  meta.timestamp, thermal);
  pthread_mutex_unlock(&thermal_mutex_);
}

void thermalCameraDevice::onVisibleFrameEvent(unsigned char* yuyv, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, const evo::IRArray<evo::IREventData>& events, void* arg)
{
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "onVisibleFrameEvent callback");
  
  if(!showVisibleChannel_) return;
  pthread_mutex_lock(&yuyv_mutex_);
  imgYUYV_ = new Image<unsigned char>(w, h,  meta.timestamp, yuyv);
  pthread_mutex_unlock(&yuyv_mutex_);
}


void thermalCameraDevice::onRawFrame(unsigned char* data, int len, evo::IRDevice* dev)
{
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "onRawFrame callback");
  
  imager_->process(data);
}

void thermalCameraDevice::onProcessExit(void* arg)
{
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "onProcessExit callback");

  pthread_mutex_lock(&thermal_mutex_);
  
  if(imgThermal_)
  {
    qThermal_.push(imgThermal_);
    imgThermal_ = NULL;
  }

  pthread_mutex_unlock(&thermal_mutex_);

  pthread_mutex_lock(&yuyv_mutex_);

  if(imgYUYV_)
  {
    //  qYUYV_.push(imgYUYV_);
    imgYUYV_    = NULL;
  }
  pthread_mutex_unlock(&yuyv_mutex_);

  pthread_mutex_lock(&float_mutex_);

  if(imgFloat_)
  {
    qFloat_.push(imgFloat_);
    imgFloat_   = NULL;
  }
  
  pthread_mutex_unlock(&float_mutex_);
}