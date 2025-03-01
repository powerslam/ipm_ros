#include "ipm.h"

IPM::IPM(): it(nh),
    roll(0), yaw(0), fov(0), x(0), y(0), z(0),
    src_w(0), src_h(0), dst_w(0), dst_h(0)
{
    this->f = boost::bind(&IPM::cameraExtrinsicCallback, this, _1, _2);
    this->server.setCallback(f);

    this->res_pub = this->it.advertise("/ipm_result", 1);
    this->raw_sub = this->it.subscribe("/camera/image_raw", 1, &IPM::imageCallback, this);
}

void IPM::buildIPMTable(){
    if(this->raw_img.empty()){
        return;
    }

    if(this->table_x.empty() || this->table_y.empty()){
        this->src_w = this->raw_img.cols;
        this->src_h = this->raw_img.rows;

        this->dst_w = 400;
        this->dst_h = 400;

        this->table_x = cv::Mat::zeros(this->raw_img.size(), CV_32FC1);
        this->table_y = cv::Mat::zeros(this->raw_img.size(), CV_32FC1);
    }

    double alpha2 = this->fov;
    double alpha = alpha2 * 0.5;
    double _factor_w = alpha2 / (this->src_w - 1);
    double _factor_h = alpha2 / (this->src_h - 1);

    this->world_x_min = this->z / tan(this->roll - alpha) * cos(this->yaw - alpha) + this->x;
    this->world_y_min = this->z / tan(this->roll - alpha) * sin(this->yaw - alpha) - this->y;

    this->world_x_max = this->z / tan(this->roll - alpha + this->src_h * _factor_h) * cos(this->yaw - alpha + this->src_w * _factor_w) + this->x;
    this->world_y_max = this->z / tan(this->roll - alpha + this->src_h * _factor_h) * sin(this->yaw - alpha + this->src_w * _factor_w) - this->y;

    // for(int y = 0; y < dst_h; y++){
    //     for(int x = 0; x < dst_w; x++){
    //         int u = (atan((this->z * sin(atan2(y, x) / (y - this->y)))) - (this->roll - alpha)) * (dst_h - 1) / alpha2;
    //         int v = (atan((y - this->y) / (x - this->x)) - (this->yaw - alpha)) * (dst_w - 1) / alpha2;
    //     }
    // }

    ROS_INFO_STREAM("\nworld_x_min : " << world_x_min << ", world_y_min : " << world_y_min << "\nworld_x_max : " << world_x_max << ", world_y_max : " << world_y_max);
}

bool IPM::ipm(){
    if(this->table_x.empty() || this->table_y.empty())
        return false;

    return true;
}

void IPM::cameraExtrinsicCallback(ipm::CameraConfig &config, uint32_t level){
    this->roll = config.roll * DEG2RAD;
    this->yaw = config.yaw * DEG2RAD;
    
    this->fov = config.fov * DEG2RAD;

    this->x = config.x;
    this->y = config.y;
    this->z = config.z;

    this->buildIPMTable();
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
