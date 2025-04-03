#include "realsenseCameraDevice.hpp"
#include <cstring>

realsenseCameraDevice::realsenseCameraDevice()
{

}

realsenseCameraDevice::~realsenseCameraDevice()
{

}


void realsenseCameraDevice::initRealsense()
{   
    /* Camera stream */
    cfgVision_.enable_stream(RS2_STREAM_COLOR,    REALSENSE_XRES, REALSENSE_YRES, RS2_FORMAT_RGB8, REALSENSE_FPS);
    cfgVision_.enable_stream(RS2_STREAM_INFRARED, REALSENSE_XRES, REALSENSE_YRES, RS2_FORMAT_Y8,   REALSENSE_FPS);
    cfgVision_.enable_stream(RS2_STREAM_DEPTH,    REALSENSE_XRES, REALSENSE_YRES, RS2_FORMAT_Z16,  REALSENSE_FPS);

    /* Pose stream */
    cfgAccel_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfgAccel_.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);

    pipeAccel_.start(cfgAccel_, [&](rs2::frame frame)
    {
        if(frame.as<rs2::motion_frame>())
        {
            auto motion = frame.as<rs2::motion_frame>();

            // Stream that bypass synchronization (such as IMU) will produce single frames
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                double ts = motion.get_timestamp();
                rs2_vector gyro_data = motion.get_motion_data();
                processGyro(gyro_data, ts);
            }

            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                rs2_vector accel_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                processAccel(accel_data);
            }

        }
    });

    profileVision_  = pipeVision_.start(cfgVision_);
}


dataFrames realsenseCameraDevice::getImagesData()
{
    rs2::frameset frameset = pipeVision_.wait_for_frames();

    rs2::align align_to_color(RS2_STREAM_COLOR);
    frameset = align_to_color.process(frameset);
    
    rs2::frame color = frameset.get_color_frame();
    rs2::frame depth = frameset.get_depth_frame();

    rs2::device dev      = profileVision_.get_device();
    rs2::depth_sensor ds = dev.query_sensors().front().as<rs2::depth_sensor>();
    float deph_scale     = ds.get_depth_scale();

    dataFrames obtained_frame;
    std::memcpy(&obtained_frame.color_data[0], color.get_data(), color.get_data_size());
    std::memcpy(&obtained_frame.depth_data[0], depth.get_data(), depth.get_data_size());
    obtained_frame.depth_scale = deph_scale;
    obtained_frame.timestamp   = frameset.get_timestamp();

    return obtained_frame;
}


void realsenseCameraDevice::processGyro(rs2_vector gyro_data, double ts)
{
    // On the first iteration, use only data from accelerometer to set the camera's initial position
    if (accel_first_) 
    {
        accel_last_ts_gyro_ = ts;
        return;
    }
    
    // Holds the change in angle, as calculated from gyro
    xyzVector gyro_angle;

    // Initialize gyro_angle with data from gyro
    gyro_angle.x = gyro_data.x; // Pitch
    gyro_angle.y = gyro_data.y; // Yaw
    gyro_angle.z = gyro_data.z; // Roll

    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro      = (ts - accel_last_ts_gyro_) / 1000.0;
    accel_last_ts_gyro_ = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * dt_gyro;

    // Apply the calculated change of angle to the current angle (theta)
    accel_theta_.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
}

void realsenseCameraDevice::processAccel(rs2_vector accel_data)
{
    // Holds the angle as calculated from accelerometer data
    xyzVector accel_angle;

    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(accel_data.y, accel_data.z);
    accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

    
    if (accel_first_)
    {
        accel_first_ = false;
        accel_theta_ = accel_angle;
        
        // We'll use PI as a convention for the initial pose
        accel_theta_.y = PI;
    }
    else
    {
        accel_theta_.x = accel_theta_.x * accel_alpha_ + accel_angle.x * (1 - accel_alpha_);
        accel_theta_.z = accel_theta_.z * accel_alpha_ + accel_angle.z * (1 - accel_alpha_);
    }
}


xyzVector realsenseCameraDevice::getTheta()
{
    return accel_theta_;
}
