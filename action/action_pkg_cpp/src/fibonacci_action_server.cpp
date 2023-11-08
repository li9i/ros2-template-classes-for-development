#include "fibonacci_action_server.hpp"

namespace action_pkg_cpp
{
  /*****************************************************************************
  */
  FibonacciActionServer::FibonacciActionServer(const rclcpp::NodeOptions& options) :
    Node("fibonacci_action_server", options)
  {
    // An action server requires 6 things:
    // The templated action type name (Fibonacci, which has been typedef-ed)
    // A ROS 2 node to add the action to (this)
    // The action name: ('fibonacci')
    // A callback function for handling goals (handle_goal)
    // A callback function for handling cancellation (handle_cancel)
    // A callback function for handling goal accept (handle_accept)
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this, "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal,
        this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FibonacciActionServer::handle_cancel,
        this, std::placeholders::_1),
      std::bind(&FibonacciActionServer::handle_accepted,
        this, std::placeholders::_1));
  }


  /*****************************************************************************
  */
  rclcpp_action::GoalResponse FibonacciActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received goal request with order %d", goal->order);
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  /*****************************************************************************
  */
  rclcpp_action::CancelResponse FibonacciActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received request to cancel goal");
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }


  /*****************************************************************************
   * This needs to return quickly to avoid blocking the executor,
   * so spin up a new thread
   */
  void FibonacciActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    std::thread
    {
      std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1),
        goal_handle
    }.detach();
  }


  /*****************************************************************************
  */
  void FibonacciActionServer::execute(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(),
      "Executing goal");

    // Construct goal, feedback, result messages
    const std::shared_ptr<const Fibonacci::Goal> goal =
      goal_handle->get_goal();
    std::shared_ptr<Fibonacci::Feedback> feedback =
      std::make_shared<Fibonacci::Feedback>();
    std::shared_ptr<Fibonacci::Result> result =
      std::make_shared<Fibonacci::Result>();

    // A reference to the feedback field
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    // Feed back intermediate results (sequence) at a rate of 1 Hz
    rclcpp::Rate loop_rate(1);
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(),
          "Goal canceled");

        return;
      }

      // Update sequence / feedback
      sequence.push_back(sequence[i] + sequence[i - 1]);

      // Publish feedback (done via reference of `feedback` from `sequence`)
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(),
        "Publishing feedback");

      loop_rate.sleep();
    }

    // At this point the goal has been calculated
    if (rclcpp::ok())
    {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),
        "Goal succeeded");
    }
  }

}  // namespace action_pkg_cpp
