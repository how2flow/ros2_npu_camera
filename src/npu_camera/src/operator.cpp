#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CamSubscriber_ : public rclcpp::Node
{
	public:
		CamSubscriber_()
			: Node("show")
		{
			setvbuf(stdout, NULL, _IONBF, BUFSIZ);
			initialize();
		}

		~CamSubscriber_()
		{
		}

	private:
		//functions
		void initialize()
		{
			auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
			auto callback =
				[this](const sensor_msgs::msg::Image::SharedPtr msg) {
					process_image(msg);
				};
			sub_ = create_subscription<sensor_msgs::msg::Image>(topic_, qos, callback);
		}

		int encoding2mat(const std::string& encoding)
		{
			if (encoding == "mono8")
				return CV_8UC1;
			else if (encoding == "bgr8")
				return CV_8UC3;
			else if (encoding == "mono16")
				return CV_16SC1;
			else if (encoding == "rgba8")
				return CV_8UC4;
			else
				std::runtime_error("Unsupported mat type");
			return 0;
		}

		void process_image(const sensor_msgs::msg::Image::SharedPtr msg)
		{
			cv::Mat frame(
					msg->height,
					msg->width,
					encoding2mat(msg->encoding),
					const_cast<unsigned char *> (msg->data.data()),
					msg->step);

			if (msg->encoding == "rgb8")
				cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

			cv::imshow(window_name, frame);
			if (cv::waitKey(1) == 'q')
				exit(0);
		}

		//vars
		std::string topic_ = "/camera/mat2image_image2mat";
		std::string window_name = "Live";

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CamSubscriber_>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
