#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/String.h"
// #include "std_srvs/Empty.h"
// #include "std_srvs/SetBool.h"
// #include "Eigen/Dense"
// #include "Eigen/Core"
// #include "Eigen/Geometry"
// #include "Eigen/StdVector"
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"
#include <string>
#include <map>
// #include <chrono>
// #include <thread>
// using namespace std;
// const double PI = 3.1415926;
std::map<int, std::string>mapErr = {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("sdk_test");

int main(int argc, char *argv[])
{
    JAKAZuRobot robot;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sdk_test");//节点名

    std::string default_ip = "10.5.5.100";
    std::string robot_ip;
    node->declare_parameter<std::string>("ip", default_ip);
    if (!node->get_parameter<std::string>("ip", robot_ip))
    {
        robot_ip = default_ip;
        RCLCPP_ERROR(LOGGER, "parameter ip not set, using default: %s", robot_ip.c_str());
    }
    if (robot.login_in(robot_ip.c_str())!=ERR_SUCC)
    {
        RCLCPP_ERROR(LOGGER, "login_in failed");
    }
    if (robot.power_on()!=ERR_SUCC)
    {
        RCLCPP_ERROR(LOGGER, "power_on failed");
    }
    if (robot.enable_robot()!=ERR_SUCC)
    {
        RCLCPP_ERROR(LOGGER, "enable_robot failed");
    }
    RobotStatus robot_status;
    robot.get_robot_status(&robot_status);
    for (int i = 0; i < 6; i++)
    {
        std::cout << robot_status.joint_position[i] << std::endl;
    }

   return 0;
                                                                               
}