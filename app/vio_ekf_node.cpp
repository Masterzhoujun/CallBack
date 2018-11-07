#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "common/parameter.h"
#include "frontend/image_processor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vio_ekf");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (!LoadParameters(n))
    {
        ROS_ERROR("Cannot initialize Image Processor...");
        return 0;
    }
    
    ImageProcessor* i_processor = new ImageProcessor(); 
    // initialize ros topic
    message_filters::Subscriber<sensor_msgs::Image> cam0_img_sub;
    message_filters::Subscriber<sensor_msgs::Image> cam1_img_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> stereo_sub(10);
  
    cam0_img_sub.subscribe(n, "cam0_image", 10);
    cam1_img_sub.subscribe(n, "cam1_image", 10);
    stereo_sub.connectInput(cam0_img_sub, cam1_img_sub);
    stereo_sub.registerCallback(&ImageProcessor::stereoCallback, i_processor);

    ros::Subscriber imu_sub = n.subscribe("imu", 50, &ImageProcessor::imuCallback, i_processor);
    
    ros::spin();
    return 1;
}