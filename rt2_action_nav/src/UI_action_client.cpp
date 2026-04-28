#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/mapstogoal.hpp"
#include <sstream>
#include <thread>

using namespace std::placeholders;

namespace nav_action
{
    class RobotNavigatorUI : public rclcpp::Node
    {
        public:
            explicit RobotNavigatorUI(const rclcpp::NodeOptions & options): Node("robot_navigator_ui", rclcpp::NodeOptions(options).use_intra_process_comms(true))
            {
                action_client_ = rclcpp_action::create_client<action_interfaces::action::Mapstogoal>(this, "map_to_goal");

                // Run the interactive menu in a background thread so the ROS executor
                // continues spinning while we block on std::getline.
                ui_thread_ = std::thread(&RobotNavigatorUI::main_loop, this);
                ui_thread_.detach();
            }
            void main_loop()
            {
                std::string input;
                bool running = true;
                float target_x = 0.0;
                float target_y = 0.0;
                float target_theta = 0.0;
                rclcpp::Rate loop_rate(10);

                while (running && rclcpp::ok())
                {
                    std::cout << "\nEnter target coordinates (x y theta), 'cancel' to cancel goal, or 'exit' to quit: " << std::flush;

                    std::getline(std::cin, input);

                    if (input == "exit")
                    {
                        running = false;
                        continue;
                    }
                    else if (input == "cancel")
                    {
                        // Check cancel BEFORE trying to parse as floats
                        action_client_->async_cancel_all_goals();
                        RCLCPP_INFO(this->get_logger(), "Cancel request sent");
                        continue;
                    }

                    std::istringstream iss(input);
                    if (!(iss >> target_x >> target_y >> target_theta))
                    {
                        std::cout << "Invalid input. Please enter in format: x y theta" << std::endl;
                        continue;
                    }
                    else
                    {
                        send_goal(target_x, target_y, target_theta);
                    }
                }
            }

        private:
            rclcpp_action::Client<action_interfaces::action::Mapstogoal>::SharedPtr action_client_{nullptr};
            rclcpp::TimerBase::SharedPtr timer_{nullptr};
            std::thread ui_thread_;
            bool goal_sent_ = false;

            void send_goal(float x, float y, float theta)
            {
                // Only send goal once
                if (goal_sent_)
                {
                    return;
                }
                
                if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    return;
                }

                auto goal_msg = action_interfaces::action::Mapstogoal::Goal();
                goal_msg.goal_coord_x = x; 
                goal_msg.goal_coord_y = y; 
                goal_msg.goal_theta = theta;

                RCLCPP_INFO(this->get_logger(), "Sending goal: (%.2f, %.2f, %.2f)", goal_msg.goal_coord_x, goal_msg.goal_coord_y, goal_msg.goal_theta);

                auto send_goal_options = rclcpp_action::Client<action_interfaces::action::Mapstogoal>::SendGoalOptions();
                send_goal_options.result_callback = std::bind(&RobotNavigatorUI::result_callback, this, _1);
                send_goal_options.feedback_callback = std::bind(&RobotNavigatorUI::feedback_callback, this, _1, _2);

                action_client_->async_send_goal(goal_msg, send_goal_options);
                goal_sent_ = true;
            }

            void feedback_callback(rclcpp_action::ClientGoalHandle<action_interfaces::action::Mapstogoal>::SharedPtr,
                                   const std::shared_ptr<const action_interfaces::action::Mapstogoal::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Received feedback: remaining distance (%.2f, %.2f)", feedback->remaining_dist_x, feedback->remaining_dist_y);
            }

            void result_callback(const rclcpp_action::ClientGoalHandle  <action_interfaces::action::Mapstogoal>::WrappedResult & result)
            {
                switch (result.code) 
                {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                }
                // Allow sending another goal after this one finishes
                goal_sent_ = false;
            }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action::RobotNavigatorUI);  