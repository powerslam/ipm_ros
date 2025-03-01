#include <ros/ros.h>

#include <ipm/CameraConfig.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <cmath>

#define DEG2RAD 0.01745329252

class IPM {
private:
    // camera extrinsic parameter
    double roll, yaw, fov, x, y, z;
    
    // raw image
    cv::Mat raw_img;

    // image information
    int src_w, src_h, dst_w, dst_h;

    // world information
    double world_x_min, world_y_min;
    double world_x_max, world_y_max;

    // ipm table for cv::remap
    cv::Mat table_x, table_y;

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
    
    void buildIPMTable();
    bool ipm();
};
