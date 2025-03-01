#include "ipm.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ipm_node");
    IPM ipm;
    ros::spin();
}
