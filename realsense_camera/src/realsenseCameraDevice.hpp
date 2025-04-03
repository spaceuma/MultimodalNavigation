#pragma once

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>   // Deproject points

#define REALSENSE_XRES 1280
#define REALSENSE_YRES 720

// Maximum frequency at which the camera can provide images
#define REALSENSE_FPS  15 // or 30

#define PI             3.1415



struct dataFrames 
{
    uint8_t color_data[REALSENSE_XRES  * REALSENSE_YRES * 3];
    uint16_t depth_data[REALSENSE_XRES * REALSENSE_YRES * 2];
    float depth_scale;
    float timestamp;
};

struct xyzVector 
{ 
    float x, y, z; 

    inline xyzVector operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    inline xyzVector operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    inline void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    inline void operator=(xyzVector other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    inline void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};

class realsenseCameraDevice
{       
    public:
        realsenseCameraDevice();
        ~realsenseCameraDevice();

        void initRealsense();
        dataFrames getImagesData();
        
        // Function to calculate the change in angle of motion based on data from gyro
        void processGyro(rs2_vector gyro_data, double ts);
        void processAccel(rs2_vector accel_data);
        xyzVector getTheta();

    private:
        // Vision and depth
        rs2::pipeline pipeVision_;
        rs2::config   cfgVision_;
        rs2::pipeline_profile profileVision_;

        // Accelerometer and gyro
        rs2::pipeline pipeAccel_;
        rs2::config   cfgAccel_;
        xyzVector accel_theta_;                 // theta is the angle of camera rotation in x, y and z components
        float     accel_alpha_        = 0.98;   // alpha indicates the part that gyro and accelerometer take in computation of theta
        bool      accel_first_        = true;
        double    accel_last_ts_gyro_ = 0;

};
