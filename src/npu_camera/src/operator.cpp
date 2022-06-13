#include "operator.h"

//functions
void CamSubscriber_::initialize()
{
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
	auto callback =
		[this](const sensor_msgs::msg::Image::SharedPtr msg) {
			process_image(msg);
		};
	sub_ = create_subscription<sensor_msgs::msg::Image>(topic_, qos, callback);
}

int CamSubscriber_::encoding2mat(const std::string& encoding)
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

void CamSubscriber_::process_image(const sensor_msgs::msg::Image::SharedPtr msg)
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
