#include "frontend/image_processor.h"

ImageProcessor::ImageProcessor() : 
    first_image(true)
{
    imu_msg_buffer.reserve(200);
}

ImageProcessor::~ImageProcessor()
{}

void ImageProcessor::stereoCallback(const sensor_msgs::ImageConstPtr& cam0_img,
                                    const sensor_msgs::ImageConstPtr& cam1_img)
{
    cout << "==================================" << endl;
    cout << "get new image" << endl;

    // image message ptr, transform RGB to gray image
    cam0_curr_img_ptr = cv_bridge::toCvShare(cam0_img, sensor_msgs::image_encodings::MONO8);
    cam1_curr_img_ptr = cv_bridge::toCvShare(cam1_img, sensor_msgs::image_encodings::MONO8);
}

// save imu_message to imu buffer
void ImageProcessor::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    cout << "get imu data" << endl;
    // wait for the first image to be set
    if (first_image)
        return;
    imu_msg_buffer.push_back(*msg);
}