#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jkerr.h"
#include "jaka_planner/jktypes.h"

#include <iostream>
#include <thread>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std;
const  double PI = 3.1415926;

class JAKA_Control : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    explicit JAKA_Control(std::string name) : Node(name)
    {
        std::string default_ip = this->declare_parameter<std::string>("ip", "10.5.5.100");
        if (!this->get_parameter<std::string>("ip", default_ip))
        {
            RCLCPP_ERROR(this->get_logger(), "parameter ip not set, using default: %s", default_ip.c_str());
            return ;
        }

        std::string default_model = this->declare_parameter<std::string>("model", "zu7");
        if (!this->get_parameter<std::string>("model", default_model))
        // if (this->get_parameter("ip", robot_ip).as_string().empty())
        {
            RCLCPP_ERROR(this->get_logger(), "parameter model not set, using default: %s", default_model.c_str());
            return ;
        }
        
        if(robot_.login_in(default_ip.c_str())!=ERR_SUCC)
        {
            RCLCPP_ERROR(this->get_logger(), "login_in failed");
        }
        // robot.set_status_data_update_time_interval(100);
        robot_.servo_move_enable(false);
        rclcpp::sleep_for(std::chrono::milliseconds(500)); 
        // Set filter parameter
        robot_.servo_move_use_joint_LPF(0.5);
        robot_.power_on();
        robot_.enable_robot();

        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this, "/jaka_"+default_model+"_controller/follow_joint_trajectory",
            // this, "/manipulator_controller/follow_joint_trajectory",
            std::bind(&JAKA_Control::handle_goal, this, _1, _2),
            std::bind(&JAKA_Control::handle_cancel, this, _1),
            std::bind(&JAKA_Control::handle_accepted, this, _1));
        
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        joint_state_timer_ = this->create_wall_timer(8ms, std::bind(&JAKA_Control::joint_states_callback, this));
    }

    ~JAKA_Control(){}

private:
    //Send the joint value of the physical robot to move_group
    void joint_states_callback()
    {
        sensor_msgs::msg::JointState joint_position;
        RobotStatus robotstatus;
        robot_.get_robot_status(&robotstatus);
        for (int i = 0; i < 6; i++)
        {
            joint_position.position.push_back(robotstatus.joint_position[i]);
            int j = i + 1;
            joint_position.name.push_back("joint_" + to_string(j));
        }
        joint_position.header.stamp = this->now(); //rclcpp::Clock().now()不能返回仿真时间
        joint_states_pub_->publish(joint_position);
    }

    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "---handle goal: %d" , goal->trajectory.joint_names.size());
        RCLCPP_INFO(this->get_logger(), "frame_id: %s %ds %dns", 
            goal->trajectory.header.frame_id.c_str(), 
            goal->trajectory.header.stamp.sec, 
            goal->trajectory.header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "trajectory size: %d", goal->trajectory.points.size());

        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&JAKA_Control::execute_move, this, _1), goal_handle}.detach();
    }

    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {   
        robot_.servo_move_enable(true);
        const auto torso_goal = goal_handle->get_goal();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        int point_num=torso_goal->trajectory.points.size();
        RCLCPP_INFO(this->get_logger(), "number of points: %d", point_num);
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
            float Duration = torso_goal->trajectory.points[i].time_from_start.sec + 
                torso_goal->trajectory.points[i].time_from_start.nanosec / 1e9;

            float dt = Duration-lastDuration;
            lastDuration = Duration;

            int step_num = int (dt/0.008); // 8ms控制周期
            int sdk_res = robot_.servo_j(&joint_pose, MoveMode::ABS, step_num);

            if (sdk_res != ERR_SUCC)
            {
                RCLCPP_INFO(this->get_logger(), "Servo_j Motion Failed");
            } 
            RCLCPP_INFO(this->get_logger(), "The return status of servo_j:%d",sdk_res);
            RCLCPP_INFO(this->get_logger(), "Accepted joint angle: %f %f %f %f %f %f %f %d", joint_pose.jVal[0],joint_pose.jVal[1],joint_pose.jVal[2],joint_pose.jVal[3],joint_pose.jVal[4],joint_pose.jVal[5],dt,step_num);
        }
    
        while(rclcpp::ok())
        {
            if(jointStates(joint_pose))
            {
                robot_.servo_move_enable(false);
                RCLCPP_INFO(this->get_logger(), "Servo Mode Disable");
                RCLCPP_INFO(this->get_logger(), "==============Motion stops or reaches the target position==============");
                break;
            }
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                robot_.motion_abort();
                RCLCPP_WARN(this->get_logger(), "==============Motion abort==============");
   
                robot_.servo_move_enable(false);
                RCLCPP_INFO(this->get_logger(), "Servo Mode Disable");
                cout<<"==============Motion stops or reaches the target position=============="<<endl;
                result->error_code = result->INVALID_GOAL;
                result->error_string = "has cancel";
                goal_handle->canceled(result);
                return; 
            }
            // rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        result->error_code = result->SUCCESSFUL;
        result->error_string = "SUCCESSFUL";
        goal_handle->succeed(result);   
        rclcpp::sleep_for(std::chrono::milliseconds(500)); 
    }

    // Determine if the robot has reached the target position.
    bool jointStates(JointValue joint_pose)
    {
        RobotStatus robotstatus;
        robot_.get_robot_status(&robotstatus);
        bool joint_state = true;
    
        for (int i = 0; i < 6; i++)
        {
            bool ret = 
                joint_pose.jVal[i] * 180 / PI - 0.2 < robotstatus.joint_position[i] * 180 / PI && 
                joint_pose.jVal[i] * 180 / PI + 0.2 > robotstatus.joint_position[i] * 180 / PI;
            joint_state = joint_state && ret; 
        }
        cout << "Whether the robot has reached the target position: " << joint_state << endl;       //1到达；0未到达
        return joint_state;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;

    JAKAZuRobot robot_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JAKA_Control>("jaka_control"));
    rclcpp::shutdown();
    return 0;
}
