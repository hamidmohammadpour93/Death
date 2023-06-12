#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <termios.h>
#include <unistd.h>
#include <termios.h> // for terminal I/O
#include <unistd.h> // for STDIN_FILENO
#include <fcntl.h> // for file control options
using namespace std::chrono_literals;
int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

class ActionClientNode : public rclcpp::Node
{
public:
    using Nav2Pose = irobot_create_msgs::action::NavigateToPosition;
    using GoalHandleDrivePosition = rclcpp_action::ClientGoalHandle<Nav2Pose>;

    ActionClientNode() : Node("action_client_node")
    {
        action_client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_position");
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

        // Below variables are given by default
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

        // Loop until the goal is completed or canceled
        while (rclcpp::ok())
        {
            if (kbhit()) // Check if a key is pressed
            {
                char c = getchar(); // Read the key pressed
                if (c == 'q')     // Check if 'q' is pressed (you can change the key as desired)
                {
                    auto navigation_goal_handle = future_goal_handle.get();
                    if (navigation_goal_handle)
                    {
                        auto future_cancel = action_client_->async_cancel_goal(navigation_goal_handle);

                        if (rclcpp::spin_until_future_complete(action_client_, future_cancel) !=
                            rclcpp::FutureReturnCode::SUCCESS)
                        {
                            RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
                        }
                        else
                        {
                            RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                            break;
                        }
                    }
                }
            }
        }
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
            // Ask the user if they want to enter another position or finish
            char choice;
            std::cout << "Do you want to enter another position? (y/n): ";
            std::cin >> choice;
            if (choice == 'y' || choice == 'Y')
            {
                sendGoalRequest(); // Send another goal request
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                rclcpp::shutdown(); // Terminate the node
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

        // Display the feedback every 0.1 seconds
        static auto prev_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - prev_time >= 0.1s)
        {
            prev_time = current_time;
 
            // Access the feedback data and display it
            RCLCPP_INFO(this->get_logger(), "Navigate State: %d", feedback->navigate_state);
            RCLCPP_INFO(this->get_logger(), "Remaining Angle Travel: %f", feedback->remaining_angle_travel);
            RCLCPP_INFO(this->get_logger(), "Remaining Travel Distance: %f", feedback->remaining_travel_distance);
        }
    }

    rclcpp_action::Client<Nav2Pose>::SharedPtr action_client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionClientNode>());
    rclcpp::shutdown();
    return 0;
}
