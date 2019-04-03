#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <cmath>
#include <algorithm>
#include <vector>
#define MARGIN 0
#define ACTIVE_PIXEL_THRESHOLD 10
#define MAX_INTENSITY 255

static const std::string OPENCV_WINDOW1 = "Image window 1";
static const std::string OPENCV_WINDOW2 = "Image window 2";
static const std::string OPENCV_WINDOW3 = "Image window 3";


void birdsEyeView(cv::Mat& img, cv::Mat& res)
{
	cv::Mat roi(img, cv::Rect(0, 260, 640, 220) ), warp, X;
	cv::Point2f src[] = {cv::Point2f(160, 290), cv::Point2f(480, 290), cv::Point2f(640, 440), cv::Point2f(0, 440)};
	cv::Point2f dst[] = {cv::Point2f(0, 0), cv::Point2f(640, 0), cv::Point2f(640, 440), cv::Point2f(0, 440)};
	cv::cvtColor(img, img, CV_BGR2GRAY);
	cv::Mat M = cv::getPerspectiveTransform(src, dst);
	cv::Mat M_inv = cv::getPerspectiveTransform(dst, src);


	cv::warpPerspective(img, warp, M, cv::Size(img.cols, img.rows));

	cv::GaussianBlur(warp, X, cv::Size(5,5), 1.5, 1.5);

	res = cv::Mat::zeros(X.size(), CV_8UC1);

	cv::Mat kernel_x = (cv::Mat_<float>(1,3) << -1, 0, 1);
	cv::filter2D(X, res, X.depth(), kernel_x);

	//intensity drops off linearily to the lower bound, so the x coordinate of
	//the beginning of the line can be determined more accurately by the
	//histogram method
	const int LOWER_DROPOFF_BOUND = 50;
	for(int i = res.rows - 1; i > 0; --i)
	{
		for(int j = MARGIN; j < res.cols - MARGIN; ++j)
		{
			if(res.at<uchar>(i, j) >= ACTIVE_PIXEL_THRESHOLD)
			{
				res.at<uchar>(i, j) = std::max(MAX_INTENSITY - (res.rows - i)/2, LOWER_DROPOFF_BOUND);
			}
		}
	}

}

bool sort_points_value(cv::Vec2i p1, cv::Vec2i p2)
{
	return p1[1] > p2[1];
}

bool sort_points_index(cv::Vec2i p1, cv::Vec2i p2)
{
	return p1[0] < p2[0];
}

//Function creates a histogram and returns the location of the peaks
//The location of the peaks is determined by going through all the histogram 
//values sorted from biggest to smallest and checking whether the adjacent 
//values have been already visited
cv::Vec2i makeHistogram(cv::Mat& img, cv::Mat& res)
{
	res = cv::Mat::zeros(img.size(), CV_8UC3);

	std::vector< cv::Vec2i> points; //(index, value)
	std::vector< cv::Vec2i> peaks;
	cv::Point prev(0, img.rows - 1);
	int* sum = new int[img.cols];
	bool* visited = new bool[img.cols + 2];
	int maxSum = 0;
	for(int i = MARGIN; i < img.cols - MARGIN; ++i)
	{
		sum[i] = 0;
		visited[i] = false;
		for(int j = 0; j < img.rows; ++j)
		{
			sum[i] += img.at<unsigned char>(j, i);
		}
	}

	for(int i = MARGIN + 2; i < img.cols - MARGIN; ++i)
	{
		sum[i - 1] = (sum[i - 2] + sum[i - 1] + sum[i])/3;
	}

	for(int i = MARGIN; i < img.cols - MARGIN; ++i)
	{
		cv::Vec2i p(i, sum[i]);
		if(i > MARGIN && i < img.cols - MARGIN) points.push_back(p);
		maxSum = std::max(sum[i], maxSum);
	}

	std::sort(points.begin(), points.end(), sort_points_value);

	for(size_t i = 0; i < points.size(); ++i)
	{
		cv::Vec2i p = points[i];
		if(!visited[p[0] - 1] && !visited[p[0] + 1]) //p is a peak
		{
			peaks.push_back(p);
		}
		visited[p[0]] = true;
	}

	cv::Vec2i peak1 = peaks[0], peak2 = peaks[1];
	for(int i = 2; i < peaks.size(); ++i)
	{
		if(std::abs(peak1[0] - peak2[0]) < 150) peak2 = peaks[i];
	}


	//Draw the histogram
	for(int i = MARGIN; i < img.cols - MARGIN; ++i)
	{
		cv::Point p(i, (int)((1 - (double)sum[i]/maxSum)*res.rows));	
		cv::line(res, prev, p, cv::Scalar(0, 0, 255), 2, CV_AA);		
		prev = p;
	}

	//Draw detected locations of peaks
	cv::Point p1(peak1[0], img.rows - 10), p2(peak1[0], 100);
	cv::Point p3(peak2[0], img.rows - 10), p4(peak2[0], 100);
	cv::line(res, p1, p2, cv::Scalar(0, 255, 0), 2, CV_AA);
	cv::line(res, p3, p4, cv::Scalar(0, 255, 0), 2, CV_AA);

	//the first peak is the one to the left
	if(peak1[0] > peak2[0]) std::swap(peak1,peak2);
	cv::Vec2i result(peak1[0], peak2[0]);

	return result;
	delete[] sum;
	delete[] visited;

}

//Algorithm based on optimizing the error function that
//fits a parabola through a set of points
cv::Mat polyFit(std::vector<cv::Point2f> data)
{
	const int DEGREE = 2;
	cv::Mat A(DEGREE + 1, DEGREE + 1, CV_32FC1), A_inv;
	cv::Mat B(DEGREE + 1, 1, CV_32FC1);
	std::vector< float > w(DEGREE + 2);

	for(int i = 0; i < w.size(); ++i) w[i] = 0;

	for(int i = 0; i < DEGREE + 1; ++i)
	{
		//calculate the derivatives
		for(size_t j = 0; j < data.size(); ++j)
		{
			float param = std::pow(data[j].x, i);
			w[2] += param;
			w[1] += data[j].x * param;
			w[0] += data[j].x * data[j].x * param;
			w[3] += data[j].y * param;
		}


		B.at<float>(DEGREE - i, 0) = w[3];

		for(int j = 0; j < DEGREE + 1; ++j)
		{
			A.at<float>(DEGREE - i, j) = w[j];
		}

		for(int i = 0; i < w.size(); ++i) w[i] = 0;

	}
	
	//solve the system of equations
	cv::invert(A, A_inv);
	cv::Mat W_res = A_inv * B;

	//return the three coefficients of the parabola
	return W_res;

}

//We search for the lines using mean pixel intensity value inside a rectangle
//with respect to the x coordinate and draw the parabola

void slidingWindowPixelSearch(cv::Mat& img, int X1, int X2)
{
	const int WINW = 64, WINH = 48, NWIN = 10;
	int pixels = WINH*WINW;
	int xl = X1 - WINW/2;
	int xr = X2 - WINW/2;
	std::vector<cv::Point2f> poly_data(NWIN);
	if(xl < 0) xl = 0;
	if(xr < 0) xr = 0;
	int y = img.rows - 1, sum_of_weighted_x = 0, sum_of_intensity = 0, intensity, xl_prev = xl, xr_prev = xr;
	static int *avg_x = new int[NWIN]; 
	avg_x[0] = xl + WINW/2;


	for(int k = 0; k < NWIN; ++k)
	{
		for(int i = y; i > y - WINH; --i)
		{
			for(int j = xl; j < xl + WINW; ++j)
			{
				if(img.at<uchar>(i, j) >= ACTIVE_PIXEL_THRESHOLD){
					++sum_of_intensity;
					sum_of_weighted_x += j;
				}
			}
		}

		if(sum_of_intensity == 0)
		{
			//if no lit pixels detected then continue the previous displacement
			//in a linear fashion
			if(k >= 2) avg_x[k] = 2*avg_x[k - 1] - avg_x[k - 2];
			else avg_x[k] = xl + WINW/2;
		}
		else
		{
			avg_x[k] = (int)((double)sum_of_weighted_x/sum_of_intensity);
		}

		sum_of_intensity = 0;
		sum_of_weighted_x = 0;

		poly_data[k].x = avg_x[k];
		poly_data[k].y = y;

		cv::Point p1(avg_x[k] - WINW/2, y), p2(avg_x[k] + WINW/2, y - WINH + 1);

		rectangle(img, p1, p2, cv::Scalar(255, 0, 255), 1, CV_AA);

		y -= (WINH + 1);
		xl = avg_x[k] - WINW/2;
		if(xl < 0) xl = 0;
	}

	cv::Mat poly = polyFit(poly_data);

	float a = poly.at<float>(0,0), b = poly.at<float>(1,0), c = poly.at<float>(2,0);

	int x1, x2;
	if(avg_x[0] < avg_x[NWIN - 1])
	{
		x1 = avg_x[0] - WINW/2;
		x2 = avg_x[NWIN - 1] + WINW/2;
	}
	else
	{
		x1 = avg_x[NWIN - 1] - WINW/2;
		x2 = avg_x[0] + WINW/2;
	}

	//draw parabola
	for(int i = x1; i < x2; i += 2)
	{
		cv::Point2i p(i, (int)(a*i*i + b*i + c));
		line(img, p, p, cv::Scalar(255, 0, 128), 2, CV_AA);
	}


}

//class that handles the whole structure of the program
class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	ImageConverter() : it_(nh_)
	{
		image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/line_finder/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW1);
		cv::namedWindow(OPENCV_WINDOW2);
		cv::namedWindow(OPENCV_WINDOW3);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW1);
		cv::destroyWindow(OPENCV_WINDOW2);
		cv::destroyWindow(OPENCV_WINDOW3);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr, dst_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch( cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat dst;
		cv::Mat hist;
		birdsEyeView(cv_ptr->image, dst);
		cv::Vec2i lane_x;
		lane_x = makeHistogram(dst, hist);

		static int prev_x1 = 0, prev_x2 = 0;


		prev_x1 = lane_x[0];
		prev_x2 = lane_x[1];

		slidingWindowPixelSearch(dst, prev_x1, prev_x2);


		cv::imshow(OPENCV_WINDOW3, hist);
		cv::imshow(OPENCV_WINDOW2, dst);
		cv::imshow(OPENCV_WINDOW1, cv_ptr->image);
		cv::waitKey(3);

		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "line_finder");

	ImageConverter ic;
	ros::spin();
	return 0;
}
