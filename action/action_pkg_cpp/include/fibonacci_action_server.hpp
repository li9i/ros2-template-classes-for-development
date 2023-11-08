#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_pkg_interfaces/action/fibonacci.hpp"

typedef action_pkg_interfaces::action::Fibonacci Fibonacci;
typedef rclcpp_action::ServerGoalHandle<Fibonacci> GoalHandleFibonacci;

namespace action_pkg_cpp
{
  class FibonacciActionServer : public rclcpp::Node
  {
    public:

      FibonacciActionServer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:

      rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle);

      void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

      void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  };  // class FibonacciActionServer

}  // namespace action_pkg_cpp

// https://answers.ros.org/question/371840/when-to-use-rclcppspin-or-rclcpp_components_register_node/
RCLCPP_COMPONENTS_REGISTER_NODE(action_pkg_cpp::FibonacciActionServer)
