#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


// #include "visibility_control.h"
// #include "controller_manager_msgs/srv/list_controllers.hpp"
// #include "controller_manager_msgs/srv/switch_controller.hpp"
// #include "xarm_api/xarm_driver.h"
#include "JAKAZuRobot.h"
#include "jkerr.h"
#include "jktypes.h"

namespace jaka_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JAKAHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // RCLCPP_SHARED_PTR_DEFINITIONS(JAKAHardwareInterface)

    // JAKA_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    // JAKA_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    // JAKA_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // // JAKA_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) override;

    // // JAKA_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) override;

    // JAKA_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override;

    // JAKA_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:

    std::shared_ptr<rclcpp::Node> node_;

    JAKAZuRobot robot_;
    std::string robot_ip_;
    bool initialized_=false;

    JointValue prev_cmds_;
    double diff_threshold_=0.0001;

    std::vector<double> hw_position_cmds_;
    std::vector<double> hw_position_states_;
    std::vector<double> hw_velocity_states_;
    // bool velocity_control_;
    // bool initialized_;
    // bool read_ready_;
    // bool reload_controller_;

    // long int read_cnts_;
    // long int read_failed_cnts_;
    // double read_max_time_;
    // double read_total_time_;
    
    // float prev_read_position_[7];
    // float curr_read_position_[7];
    // float curr_read_velocity_[7];
    // float curr_read_effort_[7];
    
    // https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html
    rclcpp::Duration desired_hw_read_period_s_  = rclcpp::Duration(0,1000);
    rclcpp::Duration desired_hw_write_period_s_ = rclcpp::Duration(0,1000);
    // rclcpp::Time curr_read_time_;
    rclcpp::Time last_read_time_;
    // rclcpp::Time curr_write_time_;
    rclcpp::Time last_write_time_;
    bool first_read_pass_, first_write_pass_ = true;

    // std::shared_ptr<rclcpp::Node> node_;
    // std::shared_ptr<rclcpp::Node> hw_node_;
    // xarm_api::XArmDriver xarm_driver_;

    // std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> req_list_controller_;
    // std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> res_list_controller_;
    // std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> req_switch_controller_;
    // std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> res_switch_controller_;

    // rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client_list_controller_;
    // rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_switch_controller_;

    // bool _check_cmds_is_change(float *prev, float *cur, double threshold = 0.0001);
    // bool _xarm_is_ready_read(void);
    // bool _xarm_is_ready_write(void);
    // bool _firmware_version_is_ge(int major, int minor, int revision);

    // bool _need_reset(void);

    // void _reload_controller(void);

    // void _init_ufactory_driver(void);

    // template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
    // int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res);
    static rclcpp::Logger getLogger();
};
}
