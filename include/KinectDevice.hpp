#ifndef MY_FREENECT_DEVICE_HPP_
#define MY_FREENECT_DEVICE_HPP_

#include <libfreenect/libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

#include <cstdint>


using namespace cv;
using namespace std;


namespace kinect_publisher{

class myMutex {
	public:
		myMutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};


class KinectDevice : public Freenect::FreenectDevice {
	public:
		KinectDevice(freenect_context *_ctx, int _index);
		~KinectDevice(){
			cv::destroyAllWindows();
			stopVideo();
			stopDepth();
		}

		void initializePublication(ros::NodeHandle &nh);
		
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp);
		
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp);
		
		bool getVideo(Mat& output);
		
		bool getDepth(Mat& output);

		bool retrieveRGBD(ros::NodeHandle &node, Mat &rgbMat, Mat &depthMat);

	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat rgbMat;
		Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
	    image_transport::ImageTransport *it_;
    	image_transport::Publisher pub_img_;
    	image_transport::Publisher pub_depth_;
    	uint32_t rgb_id_ = 0;
    	uint32_t depth_id_ = 0;
};

}

#endif