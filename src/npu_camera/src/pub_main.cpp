#include "img_pub.h"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CamPublisher_>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}