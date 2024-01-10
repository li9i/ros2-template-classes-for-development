#include "template_class.hpp"


/*******************************************************************************
*/
TemplateClass::TemplateClass() : Node("node_name")
{
  RCLCPP_INFO(this->get_logger(), "Init");

  // A parameter description ---------------------------------------------------
  rcl_interfaces::msg::ParameterDescriptor my_parameter_desc =
    rcl_interfaces::msg::ParameterDescriptor{};
  my_parameter_desc.description = "Description of parameter";

  // Declare parameter `my_parameter`; set its default value and description
  this->declare_parameter("my_parameter", "default_value", my_parameter_desc);

  // Get the value of `my_parameter`
  my_parameter_ = this->get_parameter("my_parameter").as_string();

  RCLCPP_INFO(this->get_logger(),
    "value of param my_parameter after get = %s", my_parameter_.c_str());

  // Set the value of `my_parameter`
  this->set_parameter(rclcpp::Parameter("my_parameter", "value_1"));

  RCLCPP_INFO(this->get_logger(),
    "value of param my_parameter after set = %s", my_parameter_.c_str());

  // A subscriber --------------------------------------------------------------
  subscription_ = this->create_subscription<std_msgs::msg::Empty>(
    "subscription_topic_name",
    10,
    std::bind(&TemplateClass::topic_callback, this, std::placeholders::_1));

  // A publisher ---------------------------------------------------------------
  publisher_ =
    this->create_publisher<std_msgs::msg::Empty>("publisher_topic_name", 10);

  // A service (server) --------------------------------------------------------
  // https://robotics.stackexchange.com/questions/88250/ros2-error-creating-a-service-server-as-a-member-function
  service_ = this->create_service<std_srvs::srv::Trigger>("service_offered_name",
    std::bind(&TemplateClass::service_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Create callback group for service client(s) -------------------------------
  cb_group_client_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create callback group for timer(s) ----------------------------------------
  cb_group_timer_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // A service client ----------------------------------------------------------
  // If more they should belong to the same callback group
  service_client_ =
    this->create_client<std_srvs::srv::Trigger>("service_called_name",
      rmw_qos_profile_services_default, cb_group_client_);

  // A timer -------------------------------------------------------------------
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TemplateClass::timer_callback, this),
    cb_group_timer_);
}


/*******************************************************************************
 * https://answers.ros.org/question/376294/ros2-async_send_request-callback/
 * https://answers.ros.org/question/402688/how-to-access-the-response-of-async-request-from-a-client/
 * Also with lambdas:
 * https://github.com/ros2/demos/blob/40e3732791c8c72215a5eb954e9e25be1a5ebb73/demo_nodes_cpp/src/services/add_two_ints_client_async.cpp#L62..L73
 */
void TemplateClass::receive_service_response(const
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  this->service_response_ptr_ = future.get();
  service_response_success_ = this->service_response_ptr_->success;
  service_response_message_ = this->service_response_ptr_->message;
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
  auto result_future = service_client_->async_send_request(request,
    std::bind(&TemplateClass::receive_service_response,
      this, std::placeholders::_1));

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
