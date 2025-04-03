#pragma once

#include <stack>

// IR device interfaces
#include "IRDevice.h"
// IR imager interfaces
#include "IRImager.h"
// Helper class for checking calibration files
#include "IRCalibrationManager.h"
// Logging interface
#include "IRLogger.h"
// Image converter
#include "ImageBuilder.h"
// Time measurement
#include "Image.h"

class thermalCameraDevice
{       
    public:

        thermalCameraDevice();
        ~thermalCameraDevice();


        bool configureCamera(std::string config_xml);
        bool stopCamera();
        long long obtainRGBThermalImage(unsigned char* &data);
        long long obtainFloatThermalImage(float* &data);

        evo::IRDeviceError processCameraData();

        int thermal_width_  = 640;
        int thermal_height_ = 480;
        int thermal_min_ = -20;
        int thermal_max_ = 100;

    private:
        void clearqThermal(std::stack<Image<unsigned short>*> &q );
        void clearqFloat(std::stack<Image<float>*> &q );

        inline static thermalCameraDevice* thisPtr_;

        evo::IRDeviceParams params_;
        evo::IRImager*      imager_;
        evo::ImageBuilder   iBuilder_;
        evo::IRDevice* dev_;
        

        enum IRImagerState
        {
            IRIMAGER_STATE_UNINITIALIZED = 0,
            IRIMAGER_STATE_ROAMING       = 1,
            IRIMAGER_STATE_ATTACHED      = 2,
            IRIMAGER_STATE_ACQUIRE       = 3
        };

        // The following state machine enables the handling of unplug/replug events.
        // The user can unplug the USB cable during runtime. As soon as the camera is replugged, the state machine will continue to acquire data.
        IRImagerState irState_ = IRIMAGER_STATE_ACQUIRE;

        bool showVisibleChannel_ = false;

        pthread_mutex_t thermal_mutex_= PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_t float_mutex_  = PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_t yuyv_mutex_   = PTHREAD_MUTEX_INITIALIZER;

        Image<unsigned char>*  imgYUYV_ = NULL;
        Image<unsigned short>* imgThermal_ = NULL;
        Image<float>* imgFloat_ = NULL;

        std::stack<Image<unsigned short>*> qThermal_;
        std::stack<Image<unsigned char>*>  qYUYV_;
        std::stack<Image<float>*> qFloat_;


        // onThermalFrame callback function wrapper to use it within the class
        void onThermalFrame(unsigned short* thermal, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg);
        static void onThermalFrame_wrapper(unsigned short* thermal, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
        {
            thisPtr_-> onThermalFrame(thermal, w, h, meta, arg);
        }


        // onVisibleFrame callback function wrapper to use it within the class
        void onVisibleFrame(unsigned char* yuyv, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg);
        static void onVisibleFrame_wrapper(unsigned char* yuyv, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
        {
            thisPtr_-> onVisibleFrame(yuyv, w, h, meta, arg);
        }


        // onFlageStateChange callback function wrapper to use it within the class
        void onFlageStateChange(evo::EnumFlagState fs, void* arg);
        static void onFlageStateChange_wrapper(evo::EnumFlagState fs, void* arg)
        {
            thisPtr_-> onFlageStateChange(fs, arg);
        }


        // onThermalFrameEvent callback function wrapper to use it within the class
        void onThermalFrameEvent(unsigned short* thermal, unsigned short* energy,unsigned int w, unsigned int h, evo::IRFrameMetadata meta, 
                                const evo::IRArray<evo::IREventData>& events, void* arg);
        static void onThermalFrameEvent_wrapper(unsigned short* thermal, unsigned short* energy,unsigned int w, unsigned int h, evo::IRFrameMetadata meta, 
                                                const evo::IRArray<evo::IREventData>& events, void* arg)
        {
           thisPtr_-> onThermalFrameEvent(thermal, energy, w, h, meta, events, arg);
        }


        // onVisibleFrameEvent callback function wrapper to use it within the class
        void onVisibleFrameEvent(unsigned char* yuyv, unsigned int w, unsigned int h, 
                                evo::IRFrameMetadata meta, const evo::IRArray<evo::IREventData>& events, void* arg);
        static void onVisibleFrameEvent_wrapper(unsigned char* yuyv, unsigned int w, unsigned int h, 
                                evo::IRFrameMetadata meta, const evo::IRArray<evo::IREventData>& events, void* arg)
        {
            thisPtr_-> onVisibleFrameEvent(yuyv, w, h, meta, events, arg);
        }


        // onRawFrame callback function wrapper to use it within the class
        void onRawFrame(unsigned char* data, int len, evo::IRDevice* dev);
        static void onRawFrame_wrapper(unsigned char* data, int len, evo::IRDevice* dev)
        {
            thisPtr_-> onRawFrame(data,len,dev);
        }


        // onProcessExit callback function wrapper to use it within the class
        void onProcessExit(void* arg);
        static void onProcessExit_wrapper(void* arg)
        {
            thisPtr_-> onProcessExit(arg);
        }


};