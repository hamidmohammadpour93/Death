#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#endif

using namespace std::chrono_literals;

#ifdef _WIN32
void setTerminalInputMode() {}
int kbhit()
{
    return _kbhit();
}
#else
void setTerminalInputMode()
{
    struct termios t;
    tcgetattr(0, &t);
    t.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &t);
    setbuf(stdin, NULL);
}
int kbhit()
{
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}
#endif

class ActionClientNode : public rclcpp::Node
{
public:
    using Nav2Pose = irobot_create_msgs::action::NavigateToPosition;
    using GoalHandleDrivePosition = rclcpp_action::ClientGoalHandle<Nav2Pose>;

    ActionClientNode() : Node("action_client_node")
    {
        action_client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_position");
        setTerminalInputMode();
        sendGoalRequest();
    }

private:
    void sendGoalRequest()
    {
        while (!action_client_->wait_for_action_server(2s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Action server not available. Retrying...");
        }

        auto goal_msg = Nav2Pose::Goal();
        // Prompt the user for the desired position
        std::cout << "Enter x: ";
        std::cin >> goal_msg.goal_pose.pose.position.x;
        std::cout << "Enter y: ";
        std::cin >> goal_msg.goal_pose.pose.position.y;
        // Prompt the user for the desired max translation speed
        std::cout << "Enter max translation speed: ";
        std::cin >> goal_msg.max_translation_speed;

        // Set additional goal fields
        goal_msg.goal_pose.pose.position.z = 0.0;
        goal_msg.goal_pose.pose.orientation.x = 0.0;
        goal_msg.goal_pose.pose.orientation.y = 0.0;
        goal_msg.goal_pose.pose.orientation.z = 0.0;
        goal_msg.goal_pose.pose.orientation.w = 1.0;
        goal_msg.achieve_goal_heading = true;

        auto send_goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&ActionClientNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&ActionClientNode::resultCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&ActionClientNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
        future = std::move(future_goal_handle);

        // Wait for the goal to complete or for cancellation
        /*while (rclcpp::ok())
        {
            if (kbhit())  // Check if a key is pressed
            {
                char c = getchar();  // Read the key pressed
                if (c == 'q')  // Check if 'q' is pressed (you can change the key as desired)
                {
                    cancellation();  // Cancel the goal
                    break;  // Exit the loop
                }
            }

            if (future.valid() && future.wait_for(0s) == std::future_status::ready)
            {
                break;  // Goal completed or canceled
            }
        }*/

        // Clean up resources and exit
        /*RCLCPP_INFO(this->get_logger(), "Exiting...");
        rclcpp::shutdown();*/
    }

    void goalResponseCallback(std::shared_future<GoalHandleDrivePosition::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by the action server.");
    }

    void resultCallback(const GoalHandleDrivePosition::WrappedResult &result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Action succeeded.");
                if (result.result)
                {
                    const auto &final_pose = result.result->pose;
                    RCLCPP_INFO(this->get_logger(), "Final position: (%f, %f, %f)", final_pose.pose.position.x,
                            final_pose.pose.position.y, final_pose.pose.position.z);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Action was aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Action was canceled.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                break;
        }
    }

    void feedbackCallback(GoalHandleDrivePosition::SharedPtr, const std::shared_ptr<const Nav2Pose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Navigate State: %d", feedback->navigate_state);
        RCLCPP_INFO(this->get_logger(), "Remaining Angle Travel: %f", feedback->remaining_angle_travel);
        RCLCPP_INFO(this->get_logger(), "Remaining Travel Distance: %f", feedback->remaining_travel_distance);

        // Additional logic based on feedback can be implemented here
    }

    void cancellation()
    {
        // Check if a goal has been sent to the action server
        if (future.valid() && future.wait_for(0s) == std::future_status::ready)
        {
            auto goal_handle = future.get();
            if (goal_handle)
            {
                // Send a cancel request to the action server
                auto future_cancel = action_client_->async_cancel_goal(goal_handle);
                if (rclcpp::spin_until_future_complete(shared_from_this(), future_cancel) !=
                        rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Goal was canceled.");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Cannot cancel goal because no goal was sent.");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Cannot cancel goal because no goal was sent.");
        }
    }

    rclcpp_action::Client<Nav2Pose>::SharedPtr action_client_;
    std::shared_future<GoalHandleDrivePosition::SharedPtr> future;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionClientNode>());
    rclcpp::shutdown();
    return 0;
}
