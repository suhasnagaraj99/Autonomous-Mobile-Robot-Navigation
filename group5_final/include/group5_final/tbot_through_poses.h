/**
 * @file tbot_through_poses.h
 * @brief Header file for the TurtleThroughPoses class.
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

using namespace std::chrono_literals;

namespace turtle{
    /**
     * @brief Turtle PS Class For Navigation through Poses
    */
    class TurtleThroughPoses: public rclcpp::Node{
        public:
        /**
         * @brief Constructor for TurtleThroughPoses class.
         * @param node_name The name of the ROS node.
         */
        TurtleThroughPoses(std::string node_name):Node(node_name){

            // Creating a subscription for receiving PoseArray goals
            goals_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>("Goals",rclcpp::SensorDataQoS(),
            std::bind(&TurtleThroughPoses::goals_cb , this , std::placeholders::_1));

            // Creating an action client for the navigate_through_poses action
            tbot_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this,"navigate_through_poses");

            // Creating a publisher for the initialpose topic with a queue size of 10
            tbot_initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

            // Sleeping for 5 seconds before setting the initial pose
            std::this_thread::sleep_for(std::chrono::seconds(5));

            // Setting the initial pose
            set_initial_pose();
        }

        private:

        /**
         * @brief Callback function for processing PoseArray goals
        */
        void goals_cb(geometry_msgs::msg::PoseArray msg);
        
        /**
         * @brief Callback function for goal response
        */
        void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr & goal_handle);
        /**
         * @brief Callback function for receiving feedback during goal execution
        */
        void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
        /**
         * @brief Callback function for receiving the result of the goal
        */
        void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult& result);
        // 
        /**
         * @brief Sends goals to the navigation stack
        */
        void send_goals();
        /**
         * @brief Sets the initial pose of the turtlebot
        */
        void set_initial_pose();

        // ==================== attributes ====================

        // Flag to indicate if goals are available
        bool have_goals_;

        // Poses of different waypoints
        geometry_msgs::msg::Pose wp1_;
        geometry_msgs::msg::Pose wp2_;
        geometry_msgs::msg::Pose wp3_;
        geometry_msgs::msg::Pose wp4_;
        geometry_msgs::msg::Pose wp5_;

        // Publisher for initial pose
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

        // Action client for sending goals
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr client_;

        // Action client for sending goals (tbot-specific)
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr tbot_client_;

        // Subscription for receiving goals
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_subscription_;

        // Publisher for initial pose 
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tbot_initial_pose_pub_;

        // Vector to store waypoints
        std::vector<geometry_msgs::msg::PoseStamped> wp_;
    };//class TurtleThroughPoses
} //namespace turtle
