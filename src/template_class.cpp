#include "template_class.hpp"


/*******************************************************************************
*/
TemplateClass::TemplateClass() : Node("node_name")
{
  RCLCPP_INFO(this->get_logger(), "Init");

  // A subscriber
  subscription_ = this->create_subscription<std_msgs::msg::Empty>(
    "subscription_topic_name",
    10,
    std::bind(&TemplateClass::topic_callback, this, std::placeholders::_1));

  // A publisher
  publisher_ =
    this->create_publisher<std_msgs::msg::Empty>("publisher_topic_name", 10);

  // https://robotics.stackexchange.com/questions/88250/ros2-error-creating-a-service-server-as-a-member-function
  // A service (server)
  service_ = this->create_service<std_srvs::srv::Trigger>("service_offered_name",
    std::bind(&TemplateClass::service_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Create callback group for service client(s)
  cb_group_client_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create callback group for timer(s)
  cb_group_timer_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // A service client.
  // If more they should belong to the same callback group
  service_client_ =
    this->create_client<std_srvs::srv::Trigger>("service_called_name",
      rmw_qos_profile_services_default, cb_group_client_);

  // A timer
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TemplateClass::timer_callback, this),
    cb_group_timer_);
}


/*******************************************************************************
*/
void TemplateClass::timer_callback()
{
  while (!service_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(),
        "client of service %sinterrupted while waiting for service to appear",
        service_client_->get_service_name());
      return;
    }
    RCLCPP_WARN(this->get_logger(),
      "waiting for service %s to appear...", service_client_->get_service_name());
  }

  // Craft request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = service_client_->async_send_request(request);

  while (rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(),
      "Service %s called", service_client_->get_service_name());
    std::future_status status = result_future.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::ready)
      break;
  }

  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(this->get_logger(),
      "Program canceled");
    return;
  }

  auto result = result_future.get();
  RCLCPP_INFO(this->get_logger(),
    "Service %s returned", service_client_->get_service_name());
}


/*******************************************************************************
* CAUTION. The message cannot be a reference in foxy
* https://answers.ros.org/question/400785/problem-with-subscriber-ros2/
*/
void TemplateClass::topic_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "topic_callback called");
}


/*******************************************************************************
*/
void TemplateClass::service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = 1;
  res->message = "success_message";
  RCLCPP_INFO(this->get_logger(),
    "Service %s called", service_->get_service_name());
}
