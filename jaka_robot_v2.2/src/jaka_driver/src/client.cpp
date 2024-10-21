// #include <cstdlib>
// #include <cstdio>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "std_srvs/srv/set_bool.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"

#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_msgs/srv/move.hpp"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include "jaka_msgs/srv/servo_move.hpp"
#include "jaka_msgs/srv/set_user_frame.hpp"
#include "jaka_msgs/srv/set_tcp_frame.hpp"
#include "jaka_msgs/srv/set_payload.hpp"
#include "jaka_msgs/srv/set_collision.hpp"
#include "jaka_msgs/srv/set_io.hpp"
#include "jaka_msgs/srv/get_io.hpp"
#include "jaka_msgs/srv/clear_error.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"

// #include <actionlib/server/simple_action_server.h>
// #include <control_msgs/FollowJointTrajectoryAction.h>
// #include <trajectory_msgs/JointTrajectory.h>

#include <string>
#include <map>
#include <chrono>
#include <thread>
using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Get two addends from terminal
    if (argc != 7)
    {
        // ROS_INFO("bbq0");
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "bbq0");
        return 1;
    }
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("linear_move_client");
    auto client = node->create_client<jaka_msgs::srv::Move>("/jaka_driver/linear_move");

    auto request = std::make_shared<jaka_msgs::srv::Move::Request>();
    // ROS_INFO("bbq3");
    RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "bbq3");
    for (int i = 0; i <6; i++)
    {
        request->pose.push_back(atof(argv[i]));
    }
    // srv.request.pose[0] = -376.0;
    // ROS_INFO("bbq4");
    // srv.request.pose[1] = atof(argv[2]);
    // ROS_INFO("bbq5");
    // srv.request.pose[2] = atof(argv[3]);
    // srv.request.pose[3] = atof(argv[4]);
    // srv.request.pose[4] = atof(argv[5]);
    // srv.request.pose[5] = atof(argv[6]);
    request->mvvelo = 100;
    request->mvacc = 100;
	// srv.request.coord_mode=0;
	// srv.request.index=0;

    while (!client->wait_for_service(1s)) // while循环给客户端1秒钟时间来搜索网络中的服务器节点。
    {
        if (!rclcpp::ok()) //未运行的话 报错
        {
            RCLCPP_ERROR(rclcpp::get_logger("linear_move_client"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "service not available, waiting again...");
    }

    // 客户端发送请求，且客户端节点spin直到它接收到响应或者失败为止
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)//响应成功
    {
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "ret: %ld", int(result.get()->ret));
    } 
    else //响应失败
    {
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "bbq6");
        RCLCPP_ERROR(rclcpp::get_logger("linear_move_client"), "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}