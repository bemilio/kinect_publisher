#include <KinectDevice.hpp>

namespace kinect_publisher{

KinectDevice::KinectDevice(freenect_context *_ctx, int _index)
	: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
	m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
	m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
	rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
	ownMat(Size(640,480),CV_8UC3,Scalar(0)) {

	for( unsigned int i = 0 ; i < 2048 ; i++) {
		float v = i/2048.0;
		v = std::pow(v, 3)* 6;
		m_gamma[i] = v*6*256;
	}
	startVideo();
	startDepth();
	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);
	
}

void KinectDevice::initializePublication(ros::NodeHandle &nh){
	image_transport::ImageTransport it(nh);
	pub_img_ = it.advertise("kinect/rgb", 1);
	pub_depth_ = it.advertise("kinect/depth", 1);
}

void KinectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
	std::cout << "RGB callback" << std::endl;
	m_rgb_mutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	rgbMat.data = rgb;
	m_new_rgb_frame = true;
	m_rgb_mutex.unlock();
};

void KinectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
	std::cout << "Depth callback" << std::endl;
	m_depth_mutex.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	depthMat.data = (uchar*) depth;
	m_new_depth_frame = true;
	m_depth_mutex.unlock();
}

bool KinectDevice::getVideo(Mat& output) {
	m_rgb_mutex.lock();
	if(m_new_rgb_frame) {
		cv::cvtColor(rgbMat, output, CV_RGB2BGR);
		cv_bridge::CvImage img_bridge;
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.seq = rgb_id_;
		rgb_id_++;
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, rgbMat);
        sensor_msgs::Image img_msg;
		img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
		pub_img_.publish(img_msg);
		m_new_rgb_frame = false;
		m_rgb_mutex.unlock();
		return true;
	} else {
		m_rgb_mutex.unlock();
		return false;
	}
}

bool KinectDevice::getDepth(Mat& output) {
	m_depth_mutex.lock();
	if(m_new_depth_frame) {
		depthMat.copyTo(output);
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.seq = depth_id_;
		depth_id_++;
		cv_bridge::CvImage img_bridge;
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, output);
        sensor_msgs::Image img_msg;
		img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
		pub_depth_.publish(img_msg);
		m_new_depth_frame = false;
		m_depth_mutex.unlock();
		return true;
	} else {
		m_depth_mutex.unlock();
		return false;
	}
}

bool KinectDevice::retrieveRGBD(ros::NodeHandle &node, Mat &rgbMat, Mat &depthMat) {
	if(m_new_rgb_frame && m_new_depth_frame){
		getVideo(rgbMat);
		getDepth(depthMat);
		cv::imshow("rgb", rgbMat);
		//depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
		cv::imshow("depth",depthMat);
		ROS_INFO("rgbd retrieved\n");
		return true;
	}else{
		return false;

	}
}

}