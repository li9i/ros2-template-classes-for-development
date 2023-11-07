#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

// CAUTION with messages' and services' names.
// They are required to be in PascalCase as file names and C++ types
// (e.g.) msg_srv_pkg_cpp::msg::CustomMsg
// and    msg_srv_pkg_cpp::srv::CustomSrv
// but their headers are transcripted in snake_case.
#include "msg_srv_pkg_cpp/msg/custom_msg.hpp"
#include "msg_srv_pkg_cpp/srv/custom_srv.hpp"

/*******************************************************************************
*/
class TemplateClass final : public rclcpp::Node
{
  public:

    TemplateClass();


  private:

    // *************************************************************************
    // Variables

    // A parameter
    std::string my_parameter_;

    // A subscriber
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;

    // A publisher
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;

    // A service (server)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_;

    // A timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback groups
    rclcpp::CallbackGroup::CallbackGroup::SharedPtr cb_group_client_;
    rclcpp::CallbackGroup::CallbackGroup::SharedPtr cb_group_timer_;


    // *************************************************************************
    // Methods

    // The callback of a subscriber
    void topic_callback(const std_msgs::msg::Empty::SharedPtr msg);

    // The callback of a timer
    void timer_callback();

    // The callback of a service server
    void service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

};
