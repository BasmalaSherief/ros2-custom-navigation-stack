#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/mapstogoal.hpp"
#include <cmath>
#include <mutex>

namespace nav_action
{
    class RobotNavigatorActionServer : public rclcpp::Node
    {
        public:
            explicit RobotNavigatorActionServer(const rclcpp::NodeOptions & options): Node("robot_navigator_action_server", rclcpp::NodeOptions(options).use_intra_process_comms(true))
            {
                using namespace std::placeholders;

                action_server_ = rclcpp_action::create_server<action_interfaces::action::Mapstogoal>(
                    this,
                    "map_to_goal",
                    std::bind(&RobotNavigatorActionServer::handle_goal, this, _1, _2),
                    std::bind(&RobotNavigatorActionServer::handle_cancel, this, _1),
                    std::bind(&RobotNavigatorActionServer::handle_accepted, this, _1)
                );

                velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

                odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "odom", 10, std::bind(&RobotNavigatorActionServer::odom_callback, this, _1));

                goal_frame_subsciber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "goal_pose", 10, std::bind(&RobotNavigatorActionServer::goal_pose_callback, this, _1));

                tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(100),
                    std::bind(&RobotNavigatorActionServer::timer_callback, this));  
            }

        private:
            rclcpp_action::Server<action_interfaces::action::Mapstogoal>::SharedPtr action_server_{nullptr}; 
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_{nullptr};
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_{nullptr};
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_frame_subsciber_{nullptr};
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            rclcpp::TimerBase::SharedPtr timer_{nullptr};
            geometry_msgs::msg::Pose current_pose_;
            geometry_msgs::msg::PoseStamped goal_pose_;
            std::shared_ptr<rclcpp_action::ServerGoalHandle<action_interfaces::action::Mapstogoal>> current_goal_handle_;

            bool has_active_goal_ = false;
            bool goal_reached_ = false;
            float goal_x_ = 0.0;
            float goal_y_ = 0.0;
            float goal_theta_ = 0.0;

            const float GOAL_TOLERANCE  = 0.2f;   // metres
            const float THETA_TOLERANCE = 0.05f;  // radians (~3 degrees)
            const float LINEAR_VELOCITY  = 0.5f;   // m/s  (capped by diff-drive plugin)
            const float ANGULAR_VELOCITY = 1.5f;   // rad/s gain
            
            std::mutex goal_mutex_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const action_interfaces::action::Mapstogoal::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "Received goal request with target (%.2f, %.2f)", goal->goal_coord_x, goal->goal_coord_y);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_interfaces::action::Mapstogoal>> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_interfaces::action::Mapstogoal>> goal_handle)
            {
                const auto goal = goal_handle->get_goal();
                {
                    std::lock_guard<std::mutex> lock(goal_mutex_);
                    goal_x_ = goal->goal_coord_x;
                    goal_y_ = goal->goal_coord_y;
                    goal_theta_ = goal->goal_theta;
                    has_active_goal_ = true;
                    goal_reached_ = false;
                }
                current_goal_handle_ = goal_handle;
                std::thread(&RobotNavigatorActionServer::execute, this, goal_handle).detach();
            }

            void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_interfaces::action::Mapstogoal>> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Executing goal: map to (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_theta_);
                rclcpp::Rate loop_rate(10);  // 10 Hz
                auto feedback = std::make_shared<action_interfaces::action::Mapstogoal::Feedback>();
                auto result = std::make_shared<action_interfaces::action::Mapstogoal::Result>();

                while (rclcpp::ok() && has_active_goal_)
                {
                    if (goal_handle->is_canceling())
                    {
                        result->reached = false;
                        goal_handle->canceled(result);
                        {
                            std::lock_guard<std::mutex> lock(goal_mutex_);
                            has_active_goal_ = false;
                        }
                        RCLCPP_INFO(this->get_logger(), "Goal canceled");
                        return;
                    }

                    float dist_x = goal_x_ - current_pose_.position.x;
                    float dist_y = goal_y_ - current_pose_.position.y;
                    float distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

                    // Extract yaw from odometry quaternion
                    tf2::Quaternion q(
                        current_pose_.orientation.x,
                        current_pose_.orientation.y,
                        current_pose_.orientation.z,
                        current_pose_.orientation.w);
                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    float theta_error = goal_theta_ - static_cast<float>(yaw);
                    while (theta_error >  M_PI) theta_error -= 2.0f * M_PI;
                    while (theta_error < -M_PI) theta_error += 2.0f * M_PI;

                    feedback->remaining_dist_x = dist_x;
                    feedback->remaining_dist_y = dist_y;
                    feedback->remaining_theta   = theta_error;
                    goal_handle->publish_feedback(feedback);

                    RCLCPP_INFO(this->get_logger(),
                        "Feedback: remaining (%.2f, %.2f) dist=%.2f theta_err=%.2f",
                        dist_x, dist_y, distance, theta_error);

                    // Goal is reached when position AND orientation are within tolerance
                    if (distance < GOAL_TOLERANCE && std::abs(theta_error) < THETA_TOLERANCE)
                    {
                        result->reached = true;
                        goal_handle->succeed(result);
                        {
                            std::lock_guard<std::mutex> lock(goal_mutex_);
                            has_active_goal_ = false;
                            goal_reached_ = true;
                        }
                        geometry_msgs::msg::Twist stop_cmd;
                        velocity_publisher_->publish(stop_cmd);
                        RCLCPP_INFO(this->get_logger(), "Goal reached!");
                        return;
                    }

                    loop_rate.sleep();
                }
            }
            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                current_pose_ = msg->pose.pose;
            }
            void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                goal_pose_ = *msg;
            }
            void timer_callback()
            {
                geometry_msgs::msg::Twist cmd_vel;  // zero-initialised by default

                {
                    std::lock_guard<std::mutex> lock(goal_mutex_);

                    if (has_active_goal_)
                    {
                        // --- Use odometry pose (map frame) directly ---
                        float dx = goal_x_ - current_pose_.position.x;
                        float dy = goal_y_ - current_pose_.position.y;
                        float distance = std::sqrt(dx * dx + dy * dy);

                        // Extract robot yaw from quaternion
                        tf2::Quaternion q(
                            current_pose_.orientation.x,
                            current_pose_.orientation.y,
                            current_pose_.orientation.z,
                            current_pose_.orientation.w);
                        tf2::Matrix3x3 mat(q);
                        double roll, pitch, yaw;
                        mat.getRPY(roll, pitch, yaw);

                        if (distance >= GOAL_TOLERANCE)
                        {
                            // Phase 1: drive toward the position goal
                            //   heading error = direction to goal - current yaw
                            float angle_to_goal = std::atan2(dy, dx);
                            float heading_error = angle_to_goal - static_cast<float>(yaw);
                            // Normalise to [-pi, pi]
                            while (heading_error >  M_PI) heading_error -= 2.0f * M_PI;
                            while (heading_error < -M_PI) heading_error += 2.0f * M_PI;

                            // Differential drive: linear.x only (linear.y is ignored by diff-drive)
                            // Reduce forward speed when not yet facing the goal
                            float alignment = std::cos(heading_error);  // 1 when aligned, 0 when 90 deg off
                            cmd_vel.linear.x  = LINEAR_VELOCITY * std::max(0.0f, alignment);
                            cmd_vel.linear.y  = 0.0;
                            cmd_vel.angular.z = ANGULAR_VELOCITY * heading_error;

                            RCLCPP_DEBUG(this->get_logger(),
                                "Navigating: dist=%.2f heading_err=%.2f", distance, heading_error);
                        }
                        else
                        {
                            // Phase 2: position reached – adjust final orientation
                            float theta_error = goal_theta_ - static_cast<float>(yaw);
                            while (theta_error >  M_PI) theta_error -= 2.0f * M_PI;
                            while (theta_error < -M_PI) theta_error += 2.0f * M_PI;

                            if (std::abs(theta_error) >= THETA_TOLERANCE)
                            {
                                cmd_vel.linear.x  = 0.0;
                                cmd_vel.linear.y  = 0.0;
                                cmd_vel.angular.z = ANGULAR_VELOCITY * theta_error;
                            }
                            // else: execute() thread will detect success and stop
                        }
                    }
                    // else (no active goal or already reached): cmd_vel stays zero
                }

                velocity_publisher_->publish(cmd_vel);
            }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nav_action::RobotNavigatorActionServer);