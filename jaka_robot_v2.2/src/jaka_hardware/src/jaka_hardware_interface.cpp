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
    robot_ip_ = "192.168.56.101";
    if(robot_.login_in(robot_ip_.c_str())!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(),  "login_in failed");
        return CallbackReturn::FAILURE;
    }
    if (robot_.servo_move_use_joint_LPF(0.5)!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(),"servo_move_use_joint_LPF failed");
        return CallbackReturn::FAILURE;
    }
    if (robot_.power_on()!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(), "power_on failed");
        return CallbackReturn::FAILURE;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (robot_.enable_robot()!=ERR_SUCC)
    {
        RCLCPP_FATAL(getLogger(), "enable_robot failed");
        return CallbackReturn::FAILURE;
    }

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

    RCLCPP_INFO(getLogger(), "[%s] System Sucessfully configured!", robot_ip_.c_str());
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

    RobotStatus robotstatus;
    robot_.get_robot_status(&robotstatus);

    for (auto i = 0u; i < hw_position_states_.size(); i++)
    {
        hw_position_states_[i] = robotstatus.joint_position[i];
        hw_velocity_states_[i] = 0;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type JAKAHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // RCLCPP_INFO(getLogger(), "TIME   %d s %d nanos", time.seconds() , time.nanoseconds());
    // RCLCPP_INFO(getLogger(), "period %d s %d nanos", period.seconds() , period.nanoseconds());
    // const auto diff = std::transform_reduce(
    //     hw_position_states_[0].cbegin(), hw_position_states_[0].cend(),
    //     hw_position_cmds_[0].cbegin(), 0.0,
    //     [](const auto d1, const auto d2) { return std::abs(d1) + std::abs(d2); }, std::minus<double>{});
    // if (diff <= trigger_joint_command_threshold_)
    // {
    //     return hardware_interface::return_type::OK;
    // }

    RCLCPP_INFO(getLogger(), "%f", hw_position_cmds_[0]);
    // JointValue joint_pose;
    // for (auto i=0; i<6 ;i++)
    // {
    //     joint_pose.jVal[i] =  hw_position_cmds_[i];
    // }
    // int sdk_res = robot_.servo_j(&joint_pose, MoveMode::ABS, step_num);

    // if (_need_reset()) {
    //     if (initialized_) reload_controller_ = true;
    //     initialized_ = false;
    //     return hardware_interface::return_type::OK;
    // }
    // initialized_ = true;
        
        // std::string pos_str = "[ ";
        // std::string vel_str = "[ ";
        // for (int i = 0; i < position_cmds_.size(); i++) { 
        //     pos_str += std::to_string(position_cmds_[i]); 
        //     pos_str += " ";
        //     vel_str += std::to_string(velocity_cmds_[i]); 
        //     vel_str += " ";
        // }
        // pos_str += "]";
        // vel_str += "]";
        // RCLCPP_INFO(LOGGER, "[%s] positon: %s, velocity: %s", robot_ip_.c_str(), pos_str.c_str(), vel_str.c_str());

        // int cmd_ret = 0;
        // if (velocity_control_) {
        //     for (int i = 0; i < velocity_cmds_.size(); i++) { 
        //         cmds_float_[i] = (float)velocity_cmds_[i];
        //     }
        //     // RCLCPP_INFO(LOGGER, "[%s] velocity: %s", robot_ip_.c_str(), vel_str.c_str());
        //     cmd_ret = xarm_driver_.arm->vc_set_joint_velocity(cmds_float_, true, VELO_DURATION);
        //     if (cmd_ret != 0) {
        //         RCLCPP_WARN(LOGGER, "[%s] vc_set_joint_velocity, ret=%d", robot_ip_.c_str(), cmd_ret);
        //     }
        // }
        // else {
        //     for (int i = 0; i < position_cmds_.size(); i++) { 
        //         cmds_float_[i] = (float)position_cmds_[i];
        //     }
        //     curr_write_time_ = node_->get_clock()->now();
        //     if (curr_write_time_.seconds() - prev_write_time_.seconds() > 1 || _check_cmds_is_change(prev_cmds_float_, cmds_float_)) {
        //         // RCLCPP_INFO(LOGGER, "[%s] positon: %s", robot_ip_.c_str(), pos_str.c_str());
        //         cmd_ret = xarm_driver_.arm->set_servo_angle_j(cmds_float_, 0, 0, 0);
        //         if (cmd_ret != 0) {
        //             RCLCPP_WARN(LOGGER, "[%s] set_servo_angle_j, ret= %d", robot_ip_.c_str(), cmd_ret);
        //         }
        //         if (cmd_ret == 0) {
        //             prev_write_time_ = curr_write_time_;
        //             for (int i = 0; i < 7; i++) { 
        //                 prev_cmds_float_[i] = (float)cmds_float_[i];
        //             }
        //         }
        //     }
        // }

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(jaka_hardware::JAKAHardwareInterface, hardware_interface::SystemInterface)
