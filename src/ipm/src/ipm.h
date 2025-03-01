#include <ros/ros.h>

#include <ipm/CameraConfig.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class IPM {
private:
    // camera extrinsic information
    double roll, yaw, fov, x, y, z;
    
    // raw image
    cv::Mat raw_img;

public:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    image_transport::Subscriber raw_sub;
    image_transport::Publisher  res_pub;

    dynamic_reconfigure::Server<ipm::CameraConfig> server;
    dynamic_reconfigure::Server<ipm::CameraConfig>::CallbackType f;

public:
    IPM();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraExtrinsicCallback(ipm::CameraConfig &config, uint32_t level);
};
