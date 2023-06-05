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

    ActionClientNode() : Node("action_client_node"), pose_array{}, current_index{0}
    {
        action_client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_position");
        int num_points;
        std::cout << "Enter the number of grid points: ";
        std::cin >> num_points;
        generatePoseArray(num_points);
    }



    void generatePoseArray(int num_points)
    {
        // create grid points array
        std::vector<double> grid_points;
        for (int i = 0; i < num_points; ++i) {
            double point = -1.0 + 2.0 * i / (num_points - 1);
            grid_points.push_back(point);
        }
        // fill pose_array with grid points
        for (int i = 0; i < num_points; ++i)
        {
            for(int j = 0; j < num_points; ++j)
            {
                geometry_msgs::msg::Pose current_pose;
                current_pose.position.x = grid_points[i];
                current_pose.position.y = grid_points[j];
                current_pose.position.z = 0.0;
                pose_array.push_back(current_pose);
            }
        }
    }

    void sendGoalRequest(int index)
    {
        if (index < 0 || index >= pose_array.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid pose index");
            return;
        }
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
        goal_msg.goal_pose.pose = pose_array[index];
        
        // Below variables are given by default 
        goal_msg.achieve_goal_heading = true;
        goal_msg.max_translation_speed = 0.3;
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

    void run()
    {
        while (rclcpp::ok())
        {
            // print menu
            std::cout << "\nEnter the index of the position to send to the server (0-" << pose_array.size()-1 << "): ";
            int index;
            std::cin >> index;

            // check if input is valid
            if (std::cin.fail())
            {
                std::cout << "Invalid input\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            // send pose request
            sendGoalRequest(index);

            // ask if the user wants to continue
            std::cout << "Send another position? (y/n): ";
            char answer;
            std::cin >> answer;
            if (answer == 'n' || answer == 'N')
            {
                break;
            }
        }
    }

private:
    rclcpp_action::Client<Nav2Pose>::SharedPtr action_client_;
    long int current_index;
    std::vector<geometry_msgs::msg::Pose> pose_array;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClientNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
