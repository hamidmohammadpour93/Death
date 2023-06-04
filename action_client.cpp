#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

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
        std::cout << "Enter z: ";
        std::cin >> goal_msg.goal_pose.pose.position.z;

        // Prompt the user for the desired orientation
        std::cout << "Enter orientation x: ";
        std::cin >> goal_msg.goal_pose.pose.orientation.x;
        std::cout << "Enter orientation y: ";
        std::cin >> goal_msg.goal_pose.pose.orientation.y;
        std::cout << "Enter orientation z: ";
        std::cin >> goal_msg.goal_pose.pose.orientation.z;
        std::cout << "Enter orientation w: ";
        std::cin >> goal_msg.goal_pose.pose.orientation.w;

        // Prompt the user for the desired max translation speed
        std::cout << "Enter max translation speed: ";
        std::cin >> goal_msg.max_translation_speed;
        goal_msg.achieve_goal_heading = true;

        auto send_goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&ActionClientNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&ActionClientNode::resultCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&ActionClientNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClientNode>();
    rclcpp::spin(node);
    return 0;
}
