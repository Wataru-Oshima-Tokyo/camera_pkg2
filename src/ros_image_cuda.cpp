#include <deque>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/cudaarithm.hpp> // Include CUDA module
#include <opencv2/cudawarping.hpp> // Include CUDA module


// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>






using namespace cv;
using std::placeholders::_1;
struct timespec start, stop;
double fstart, fstop;
static const std::string IMAGE_TOPIC = "image_raw";
static const size_t FPS_SAMPLE_SIZE = 10; // Number of frames to average for FPS calculation
std::deque<double> frame_times;
double avg_frame_time;
Mat src, dst;
cuda::GpuMat src_gpu, src_hsv_gpu; // Declare GpuMat

class imageSubscriber : public rclcpp::Node{

   public:  
        imageSubscriber() : Node("image_subscriber"){
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC,1000, std::bind(&imageSubscriber::image_callback, this, _1));

        }

    private:
    	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;


        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
        	std_msgs::msg::Header msg_header = msg->header;
        	std::string frame_id = msg_header.frame_id.c_str();
        	// ROS_INFO_STREAM("New Image from " << frame_id);

        	cv_bridge::CvImageConstPtr cv_ptr;
        	try
        	{
            		cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        	}
        	catch (cv_bridge::Exception& e)
        	{
            		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            		return;
        	}

            src = cv_ptr->image;
            dst.create(src.size(), src.type());
            
            src_gpu.upload(src); // Upload Mat to GpuMat
            cuda::cvtColor(src_gpu, src_hsv_gpu, COLOR_BGR2HSV); // Use CUDA function for color conversion
            
            src_hsv_gpu.download(src); // Download GpuMat to Mat
            
            clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
        	double frame_time = fstop - fstart;

            // Calculate moving average of frame times
            frame_times.push_back(frame_time);
            if (frame_times.size() > FPS_SAMPLE_SIZE) {
                frame_times.pop_front();
            }

            avg_frame_time = 0;
            for (const auto &time : frame_times) {
                avg_frame_time += time;
            }
            avg_frame_time /= frame_times.size();

            double fps = 1.0 / avg_frame_time;
            std::string fps_text = "FPS: " + std::to_string(fps);

        	putText(src, //target image
            		fps_text, //text
            		Point(10, 30), //top-left position
            		FONT_HERSHEY_DUPLEX,
            		1.0,
            		Scalar(118, 185, 0), //font color
            		2);
        	cv::imshow("src", src);
        	cv::waitKey(3);   
        }
};

// ...

int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imageSubscriber>());
    rclcpp::shutdown();
    destroyAllWindows();
    return 0;
}