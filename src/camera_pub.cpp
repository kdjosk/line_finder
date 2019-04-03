#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

//simple node that publishes image from a webcam 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_pub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

	int video_source = 0;

	cv::VideoCapture cap(video_source);
	if(!cap.isOpened())
	{
		return 1;
	}
	cv::Mat frame;
	sensor_msgs::ImagePtr msg;

	ros::Rate loop_rate(5);
	while(nh.ok()) {

		cap >> frame;

		if(!frame.empty()) {
			
			msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
         	pub.publish(msg);
       		cv::waitKey(1);
		}

		ros::spinOnce();
		loop_rate.sleep();

	}
}
