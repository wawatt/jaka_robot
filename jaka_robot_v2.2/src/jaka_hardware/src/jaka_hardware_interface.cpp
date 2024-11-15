#include "jaka_hardware/jaka_hardware_interface.hpp"

// #define SERVICE_CALL_FAILED 999
// #define SERVICE_IS_PERSISTENT_BUT_INVALID 998
// #define ROBOT_IS_DISCONNECTED -1
// #define WAIT_SERVICE_TIMEOUT 996
// #define VELO_DURATION 1

namespace jaka_hardware
{

rclcpp::Logger JAKAHardwareInterface::getLogger() {
    return rclcpp::get_logger("JAKAHardwareInterface");
}

CallbackReturn JAKAHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != 
        CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // 初始化
    const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
        {
        return it->second;
        }
        return default_value;
    };
    auto robot_ip = get_hardware_parameter("robot_ip", "10.5.5.100");
    if(robot_.login_in(robot_ip.c_str())!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(),  "login_in failed");
        return CallbackReturn::FAILURE;
    }
    if (robot_.power_on()!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(), "power_on failed");
        return CallbackReturn::FAILURE;
    }
    if (robot_.enable_robot()!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(), "enable_robot failed");
        return CallbackReturn::FAILURE;
    }
    robot_.servo_move_enable(false);
    robot_.servo_speed_foresight(15,0.03);
    if (robot_.servo_move_enable(true)!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(), "servo_move_enable failed");
        return CallbackReturn::FAILURE;
    }

    memset(prev_cmds_.jVal, 0, sizeof(prev_cmds_.jVal));

    hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const auto& joint : info_.joints) {

        // 一个位置控制
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(getLogger(), 
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), 
                joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' have %s command interfaces found. '%s' expected.", 
                joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), 
                hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }

        // 2个状态
        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has %zu state interface. 2 expected.", 
                joint.name.c_str(),
                joint.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", 
                joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", 
                joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }

    }

    initialized_ = true;
    RCLCPP_INFO(getLogger(), "[%s] System Sucessfully configured!", robot_ip.c_str());
    
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JAKAHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JAKAHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_cmds_[i]));
    }

    return command_interfaces;
}

// CallbackReturn JAKAHardwareInterface::on_activate(
//     const rclcpp_lifecycle::State & previous_state)
// {
//     RCLCPP_INFO(getLogger(), "Activating ...please wait...");
//     robot_.servo_move_enable(false);
//     // xarm_driver_.arm->motion_enable(true);
//     // xarm_driver_.arm->set_mode(velocity_control_ ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
//     // xarm_driver_.arm->set_state(XARM_STATE::START);

//     // req_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
//     // res_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();
//     // req_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
//     // res_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Response>();

//     // client_list_controller_ = hw_node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
//     // client_switch_controller_ = hw_node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

//     for (auto i = 0u; i < hw_position_states_.size(); i++) {
//         if (std::isnan(hw_position_states_[i])) {
//             hw_position_states_[i] = 0;
//             hw_position_cmds_[i] = 0;
//         } else {
//             hw_position_cmds_[i] = hw_position_states_[i];
//         }
//     }

//     RCLCPP_INFO(getLogger(), "[%s] System Sucessfully activated!", robot_ip_.c_str());
//     return CallbackReturn::SUCCESS;
// }

// CallbackReturn JAKAHardwareInterface::on_deactivate(
//     const rclcpp_lifecycle::State& previous_state)
// {
//     RCLCPP_INFO(getLogger(), "[%s] Stopping ...please wait...", robot_ip_.c_str());

//     // xarm_driver_.arm->set_mode(XARM_MODE::POSE);
//     robot_.servo_move_enable(false);
//     RCLCPP_INFO(getLogger(), "[%s] System sucessfully stopped!", robot_ip_.c_str());
//     return CallbackReturn::SUCCESS;
// }

hardware_interface::return_type JAKAHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    // RobotStatus robotstatus;
    // robot_.get_robot_status(&robotstatus);
    JointValue cur_jval;
    robot_.get_joint_position(&cur_jval);
    for (auto i = 0u; i < hw_position_states_.size(); i++)
    {
        hw_position_states_[i] = cur_jval.jVal[i];
        hw_velocity_states_[i] = 0;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type JAKAHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!initialized_)
    {
        return hardware_interface::return_type::OK;
    }

    bool cmds_changed = false;
    for (int i = 0; i < 6; i++) {
        if (std::abs(prev_cmds_.jVal[i] - hw_position_cmds_[i]) > diff_threshold_) 
            cmds_changed = true;
        prev_cmds_.jVal[i] = hw_position_cmds_[i];
    }

    if (cmds_changed)
    {    
        auto ret = robot_.servo_j(&prev_cmds_, MoveMode::ABS);
        if (ret!=ERR_SUCC)
        {
            // ErrorCode error_code;
            // robot_.get_last_error(&error_code);
            RCLCPP_FATAL(getLogger(), "servo_j failed: %d");
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(jaka_hardware::JAKAHardwareInterface, hardware_interface::SystemInterface)
