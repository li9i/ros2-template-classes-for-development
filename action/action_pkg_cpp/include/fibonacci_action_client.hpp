#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_pkg_interfaces/action/fibonacci.hpp"

typedef action_pkg_interfaces::action::Fibonacci Fibonacci;
typedef rclcpp_action::ClientGoalHandle<Fibonacci> GoalHandleFibonacci;

namespace action_pkg_cpp
{
  class FibonacciActionClient : public rclcpp::Node
  {
    public:

      FibonacciActionClient(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

      void send_goal();

    private:

      // The action client
      rclcpp_action::Client<Fibonacci>::SharedPtr action_client_ptr_;

      // Once the timer fires the action client will call the action server
      rclcpp::TimerBase::SharedPtr timer_;

      // callback options (functions) for goal admission, feedback, and result
      // that need to be set
      rclcpp_action::Client<Fibonacci>::SendGoalOptions send_goal_options_;


      // This function is called upon acceptance or rejection of the goal by the
      // goal server
      void goal_response_callback(
        const GoalHandleFibonacci::SharedPtr& goal_handle);

      // This function is called periodically until the action is completed
      void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback);

      // This function carries the result of the action
      void result_callback(const GoalHandleFibonacci::WrappedResult& result);

  };  // class FibonacciActionClient

}  // namespace action_pkg_cpp

// https://answers.ros.org/question/371840/when-to-use-rclcppspin-or-rclcpp_components_register_node/
RCLCPP_COMPONENTS_REGISTER_NODE(action_pkg_cpp::FibonacciActionClient)
