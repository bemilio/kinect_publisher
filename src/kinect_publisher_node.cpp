#include <kinect_publisher_node.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgb-d_publisher");
    ros::NodeHandle node;
    while(node.ok()){
	    Mat depthMat(Size(640,480),CV_16UC1);
	    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	    Freenect::Freenect freenect;
		MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	    device.retrieve_rgbd(node, rgbMat, depthMat);
	}
    return 0;
}