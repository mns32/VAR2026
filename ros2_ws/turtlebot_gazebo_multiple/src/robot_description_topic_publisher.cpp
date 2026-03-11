#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class RobotDescriptionTopicPublisher final : public rclcpp::Node
{
public:
  RobotDescriptionTopicPublisher()
  : Node("robot_description_topic_publisher")
  {
    target_node_ = this->declare_parameter<std::string>(
      "target_node", "/robot_state_publisher");
    topic_name_ = this->declare_parameter<std::string>(
      "topic", "/robot_description");
    republish_period_ms_ = this->declare_parameter<int>(
      "republish_period_ms", 1000);

    publisher_ = this->create_publisher<std_msgs::msg::String>(
      topic_name_, rclcpp::QoS(1).reliable().durability_volatile());

    client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this->get_node_base_interface(),
      this->get_node_topics_interface(),
      this->get_node_graph_interface(),
      this->get_node_services_interface(),
      target_node_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(50, republish_period_ms_)),
      std::bind(&RobotDescriptionTopicPublisher::tick, this));
  }

private:
  void tick()
  {
    if (!client_->service_is_ready()) {
      return;
    }

    if (!pending_request_) {
      pending_request_ = true;
      pending_future_ = client_->get_parameters({"robot_description"});
      return;
    }

    if (pending_future_.valid() &&
        pending_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto params = pending_future_.get();
      pending_request_ = false;

      if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        cached_description_ = params[0].as_string();
      }
    }

    if (cached_description_.empty()) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = cached_description_;
    publisher_->publish(std::move(msg));
  }

  std::string target_node_;
  std::string topic_name_;
  int republish_period_ms_{1000};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool pending_request_{false};
  std::shared_future<std::vector<rclcpp::Parameter>> pending_future_;
  std::string cached_description_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescriptionTopicPublisher>());
  rclcpp::shutdown();
  return 0;
}
