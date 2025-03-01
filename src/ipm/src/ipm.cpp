#include "ipm.h"

IPM::IPM(): it(nh),
    roll(0), yaw(0), fov(0), x(0), y(0), z(0){
    
    this->f = boost::bind(&IPM::cameraExtrinsicCallback, this, _1, _2);
    this->server.setCallback(f);

    this->res_pub = this->it.advertise("/ipm_result", 1);
    this->raw_sub = this->it.subscribe("/camera/image_raw", 1, &IPM::imageCallback, this);
}

void IPM::cameraExtrinsicCallback(ipm::CameraConfig &config, uint32_t level){
    this->roll = config.roll;
    this->yaw = config.yaw;
    
    this->fov = config.fov;

    this->x = config.x;
    this->y = config.y;
    this->z = config.z;
}

void IPM::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        this->raw_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::imshow("Camera Feed", this->raw_img);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
