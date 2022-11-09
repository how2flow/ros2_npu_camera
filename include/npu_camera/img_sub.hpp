#ifndef _IMG_SUB_HPP_
#define _IMG_SUB_HPP_

class CamSubscriber_ : public rclcpp::Node
{
public:
  explicit CamSubscriber_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~CamSubscriber_();

private:
  //functions
  void initialize();
  int encoding2mat(const std::string& encoding);
  void process_image(const sensor_msgs::msg::Image::SharedPtr msg);
  std::string topic_ = "/camera/mat2image_image2mat";
  std::string window_name = "Live";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
#endif
