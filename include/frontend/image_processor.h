#ifndef __IMAGE_PROCESSOR_H__
#define __IMAGE_PROCESSOR_H__

#include <vector>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

using std::cout;
using std::endl;
using std::vector;

class ImageProcessor
{
public:
    // Constructor
    ImageProcessor();
    // Destructor
    ~ImageProcessor();

    /*
    * @brief stereoCallback
    *    Callback function for the stereo images.
    * @param cam0_img left image.
    * @param cam1_img right image.
    */
    void stereoCallback(const sensor_msgs::ImageConstPtr& cam0_img,
                        const sensor_msgs::ImageConstPtr& cam1_img);
    /*
    * @brief imuCallback
    *  Callback function for imu message.
    * @param msg IMU msg.
    */
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    
private:
    // indicate if this is the first image message.
    bool first_image;

    // imu message buffer
    vector<sensor_msgs::Imu> imu_msg_buffer;
    
    // previous and current image ptr
    cv_bridge::CvImageConstPtr cam0_prev_img_ptr;
    cv_bridge::CvImageConstPtr cam0_curr_img_ptr;
    cv_bridge::CvImageConstPtr cam1_curr_img_ptr;
};

#endif