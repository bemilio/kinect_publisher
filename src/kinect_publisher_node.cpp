#include <kinect_publisher_node.hpp>

int main(int argc, char **argv) {
	cout << "Initializing ros";
    ros::init(argc, argv, "rgb-d_publisher");
    ros::NodeHandle node;
    Mat depthMat(Size(640,480),CV_16UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	cout << "Opening device...\n";
    Freenect::Freenect freenect;

	kinect_publisher::KinectDevice& device = freenect.createDevice<kinect_publisher::KinectDevice>(0);
	device.initializePublication(node);
	cout << "starting!!!!\n";
	ros::Rate loop_rate(40);
    while(node.ok()){  
    	cout << "Retrieving rgbd \n";
	    device.retrieveRGBD(node, rgbMat, depthMat);
	    ros::spinOnce();
	    loop_rate.sleep();
	}
    return 0;
}