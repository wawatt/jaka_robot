#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/String.h"
// #include "std_srvs/Empty.h"
// #include "std_srvs/SetBool.h"
// #include "geometry_msgs/TwistStamped.h"
// #include "sensor_msgs/JointState.h"
// #include "Eigen/Dense"
// #include "Eigen/Core"
// #include "Eigen/Geometry"
// #include "Eigen/StdVector"
// #include "jaka_msgs/RobotMsg.h"
// #include "jaka_msgs/Move.h"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include "jaka_msgs/srv/servo_move.hpp"
// #include "jaka_msgs/SetUserFrame.h"
// #include "jaka_msgs/SetTcpFrame.h"
// #include "jaka_msgs/SetPayload.h"
// #include "jaka_msgs/SetCollision.h"
// #include "jaka_msgs/ClearError.h"
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"
// #include <string>
using namespace std;

BOOL in_pos;
JAKAZuRobot robot;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("client_test");
    auto servo_move_enable_client =  node->create_client<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable");
    auto servo_j_client = node->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_j");
    
    while (!servo_move_enable_client->wait_for_service(1s)) // while循环给客户端1秒钟时间来搜索网络中的服务器节点。
    {
        if (!rclcpp::ok()) //未运行的话 报错
        {
            RCLCPP_ERROR(rclcpp::get_logger("servo_move_enable_client"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("servo_move_enable_client"), "service not available, waiting again...");
    }
    while (!servo_j_client->wait_for_service(1s)) // while循环给客户端1秒钟时间来搜索网络中的服务器节点。
    {
        if (!rclcpp::ok()) //未运行的话 报错
        {
            RCLCPP_ERROR(rclcpp::get_logger("servo_j_client"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("servo_j_client"), "service not available, waiting again...");
    }

    auto enable_state_request = std::make_shared<jaka_msgs::srv::ServoMoveEnable::Request>();
    enable_state_request->enable = TRUE;
    auto res1 = servo_move_enable_client->async_send_request(enable_state_request);
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    auto servo_pose_request = std::make_shared<jaka_msgs::srv::ServoMove::Request>();
    float pose[6] = {0.001, 0, 0, 0, 0, 0.001};
    for (int i =0; i < 6; i++)
    {
        servo_pose_request->pose.push_back(pose[i]);
    } 

    for (int i = 0; i < 200; i++)
    {
        auto res2 = servo_j_client->async_send_request(servo_pose_request);
        if (rclcpp::spin_until_future_complete(node, res2) == rclcpp::FutureReturnCode::SUCCESS)//响应成功
        {
            auto result = res2.get();
            RCLCPP_INFO(rclcpp::get_logger("servo_j_client"), "The return value of calling servo_j: %d" , result->ret);
            RCLCPP_INFO(rclcpp::get_logger("servo_j_client"), "%s", result->message.c_str());
        } 
        else //响应失败
        {
            RCLCPP_ERROR(rclcpp::get_logger("servo_j_client"), "Failed to call service");
        }
  
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
    return 0;
}
