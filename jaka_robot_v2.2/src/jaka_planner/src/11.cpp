#include "rclcpp/rclcpp.hpp"
#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jkerr.h"
#include "jaka_planner/jktypes.h"
#include <sensor_msgs/msg/joint_state.hpp>
// #include <actionlib/server/simple_action_server.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <string>
#include <map>
// #include "std_srvs/Empty.h"
// #include "std_srvs/SetBool.h"
// #include "std_msgs/Empty.h"
#include <thread>

using namespace std;
JAKAZuRobot robot;
const  double PI = 3.1415926;
BOOL in_pos;
int ret_preempt;
int ret_inPos;

// typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

map<int, string>mapErr = {
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

// Determine if the robot has reached the target position.
bool jointStates(JointValue joint_pose)
{
    RobotStatus robotstatus;
    robot.get_robot_status(&robotstatus);
    bool joint_state = true;
   
    for (int i = 0; i < 6; i++)
    {
        bool ret = joint_pose.jVal[i] * 180 / PI - 0.2 < robotstatus.joint_position[i] * 180 / PI
        && robotstatus.joint_position[i] * 180 / PI < joint_pose.jVal[i] * 180 / PI + 0.2;
        joint_state = joint_state && ret; 
    }
    cout << "Whether the robot has reached the target position: " << joint_state << endl;       //1到达；0未到达
    return joint_state;
}

//Moveit server
void goalCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& torso_goal, Server* as)
{
    BOOL in_pos;
    robot.servo_move_enable(true);
    int point_num=torso_goal->trajectory.points.size();
    ROS_INFO("number of points: %d",point_num);
    JointValue joint_pose;
    float lastDuration=0.0;
    OptionalCond* p = nullptr;
    for (int i=1; i<point_num; i++) {        
        joint_pose.jVal[0] = torso_goal->trajectory.points[i].positions[0];
        joint_pose.jVal[1] = torso_goal->trajectory.points[i].positions[1];
        joint_pose.jVal[2] = torso_goal->trajectory.points[i].positions[2];
        joint_pose.jVal[3] = torso_goal->trajectory.points[i].positions[3];
        joint_pose.jVal[4] = torso_goal->trajectory.points[i].positions[4];
        joint_pose.jVal[5] = torso_goal->trajectory.points[i].positions[5];      
        float Duration=torso_goal->trajectory.points[i].time_from_start.toSec();

        float dt=Duration-lastDuration;
        lastDuration=Duration;

        int step_num=int (dt/0.008);
        int sdk_res=robot.servo_j(&joint_pose, MoveMode::ABS, step_num);

        if (sdk_res !=0)
        {
            ROS_INFO("Servo_j Motion Failed");
        } 
        ROS_INFO("The return status of servo_j:%d",sdk_res);
        ROS_INFO("Accepted joint angle: %f %f %f %f %f %f %f %d", joint_pose.jVal[0],joint_pose.jVal[1],joint_pose.jVal[2],joint_pose.jVal[3],joint_pose.jVal[4],joint_pose.jVal[5],dt,step_num);
    }

    while(true)
    {
        if(jointStates(joint_pose))
        {
            robot.servo_move_enable(false);
            ROS_INFO("Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }

        if ( ret_preempt = as->isPreemptRequested())      
        {
            robot.motion_abort();
            robot.servo_move_enable(false);
            ROS_INFO("Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }
        ros::Duration(0.5).sleep();
    }
as->setSucceeded();    
ros::Duration(0.5).sleep();
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_server_log");

class MoveitServer : public rclcpp::Node
{
public: 
    MoveitServer() : Node("moveit_server")
    {
        string default_ip = "10.5.5.100";
        string robot_ip;
        this->declare_parameter<std::string>("ip", default_ip);
        if (!this->get_parameter<std::string>("ip", robot_ip))
        {
            robot_ip = default_ip;
            RCLCPP_ERROR(LOGGER, "parameter ip not set, using default: %s", robot_ip.c_str());
            return ;
        }
        string default_model = "zu7";
        string robot_model;
        this->declare_parameter<std::string>("model", default_model);
        if (!this->get_parameter<std::string>("model", robot_model))
        // if (this->get_parameter("ip", robot_ip).as_string().empty())
        {
            robot_model = default_model;
            RCLCPP_ERROR(LOGGER, "parameter model not set, using default: %s", robot_model.c_str());
            return ;
        }
        
        if(robot.login_in(robot_ip.c_str())!=ERR_SUCC)
        {
            RCLCPP_ERROR(LOGGER, "login_in failed");
        }
        // robot.set_status_data_update_time_interval(100);
        robot.servo_move_enable(false);
        rclcpp::sleep_for(std::chrono::milliseconds(500)); 
        // Set filter parameter
        robot.servo_move_use_joint_LPF(0.5);
        robot.power_on();
        robot.enable_robot();
        //Create topic "/joint_states"
        joint_position_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        joint_state_timer_ = this->create_wall_timer(8ms, std::bind(&JakaDriver::joint_states_callback, this));
        
        //Create action server object
        Server moveit_server(nh, "/jaka_"+robot_model+"_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    }

private:
    //Send the joint value of the physical robot to move_group
    void joint_states_callback()
    {
        sensor_msgs::msg::JointState joint_position;
        RobotStatus robotstatus;
        robot.get_robot_status(&robotstatus);
        for (int i = 0; i < 6; i++)
        {
            joint_position.position.push_back(robotstatus.joint_position[i]);
            int j = i + 1;
            joint_position.name.push_back("joint_" + to_string(j));
        }
        joint_position.header.stamp = this->now(); //rclcpp::Clock().now()不能返回仿真时间
        joint_states_pub_.publish(joint_position);
    }


    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_position_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    

}

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);


    // robot.set_status_data_update_time_interval(100);
    ros::Rate rate(125);



    Server moveit_server(nh, "/jaka_"+robot_model+"_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
	moveit_server.start();
    cout << "==================Moveit Start==================" << endl;

    while(ros::ok())
    {
        //Report robot joint information to RVIZ
        joint_states_callback(joint_states_pub);
        rate.sleep();
        ros::spinOnce();
    }
    //ros::spin();
}