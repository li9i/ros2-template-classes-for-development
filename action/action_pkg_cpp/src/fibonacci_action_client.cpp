#include "fibonacci_action_client.hpp"

namespace action_pkg_cpp
{
  /*****************************************************************************
  * Constructor
  */
  FibonacciActionClient::FibonacciActionClient(
    const rclcpp::NodeOptions& options) : Node("fibonacci_action_client", options)
  {
    // An action client requires 3 things:
    // The templated action type name (Fibonacci, which has been typedef-ed)
    // A ROS 2 node to add the action client to (this)
    // The action name ('fibonacci')
    this->action_client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this, "fibonacci");

    // When the timer expires send_goal will be called
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));

    // Set callback functions for goal admission, feedback, and result
    send_goal_options_ = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    // This function is called upon acceptance or rejection of the goal by the
    // goal server
    send_goal_options_.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this,
        std::placeholders::_1);

    // This function is called periodically until the action is completed
    send_goal_options_.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this,
        std::placeholders::_1, std::placeholders::_2);

    // This function carries the result of the action
    send_goal_options_.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this,
        std::placeholders::_1);
  }


  /*****************************************************************************
  * Action client calls action server
  */
  void FibonacciActionClient::send_goal()
  {
    // ...as a result send_goal will only execute once and the node will stop
    this->timer_->cancel();

    // At ctrl-c
    if (!this->action_client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(),
        "Action server not available after waiting");

      rclcpp::shutdown();
    }

    // Craft goal and request
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(),
      "Sending goal");

    // Send goal
    this->action_client_ptr_->async_send_goal(goal_msg, send_goal_options_);
  }


  /*****************************************************************************
   * This function is called upon acceptance or rejection of the goal by the
   * goal server
   */
  void FibonacciActionClient::goal_response_callback(
    std::shared_future<GoalHandleFibonacci::SharedPtr> future)
  {
    auto goal_handle = future.get();

    if (goal_handle)
      RCLCPP_INFO(this->get_logger(),
        "Goal accepted by server, awaiting for result");
    else
      RCLCPP_ERROR(this->get_logger(),
        "Goal was rejected by server");
  }


  /*****************************************************************************
   * This function is called periodically until the action is completed
   */
  void FibonacciActionClient::feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";

    for (auto number : feedback->partial_sequence)
      ss << number << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }


  /*****************************************************************************
   * This function carries the result of the action
   */
  void FibonacciActionClient::result_callback(
    const GoalHandleFibonacci::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence)
      ss << number << " ";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    RCLCPP_INFO(this->get_logger(), "Action completed. Shutting down.");

    rclcpp::shutdown();
  }

}  // namespace action_pkg_cpp
