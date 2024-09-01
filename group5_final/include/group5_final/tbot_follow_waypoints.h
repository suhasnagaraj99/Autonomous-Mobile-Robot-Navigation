/**
 * @file tbot_follow_waypoints.h
 * @brief Header File for TurtleWaypoints Class 
*/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

using namespace std::chrono_literals;

namespace turtle{
    /**
     * @brief TurtleWaypoints class for handling navigation goals.
    */
    class TurtleWaypoints: public rclcpp::Node{
        public:
        /**
         * @brief Constructor for TurtleWaypoints.
         */
        TurtleWaypoints(std::string node_name):
        
        Node(node_name){

            // Create subscription for receiving PoseArray goals
            goals_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>("Goals",rclcpp::SensorDataQoS(),
            std::bind(&TurtleWaypoints::goals_cb , this , std::placeholders::_1));

            // Create action client for following waypoints
            tbot_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this,"follow_waypoints");

            // Create publisher for initial pose
            tbot_initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

            std::this_thread::sleep_for(std::chrono::seconds(5));

            // Set the initial pose
            set_initial_pose();
        
        }

        private:

        /**
         * @brief CallBack for goals
         * @param msg
        */
        void goals_cb(geometry_msgs::msg::PoseArray msg);
        
        /**
         * @brief CallBack for goals
        */   
        void send_goals();

        /**
         * @brief Function to set the initial pose of the turtlebot.
        */
        void set_initial_pose();
 
        /**
         * @brief Callback function for handling goal response.
        */
        void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr & goal_handle);

        /**
         * @brief Callback function for handling feedback during goal execution.
         */
        void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr,const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback);

        /**
         * @brief Callback function for handling the result of the goal.
         */
        void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult& result);


        // ==================== attributes ====================
        
        //Action client for the follow_waypoints action.
        
        rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr tbot_client_;
        
        // Subscription for receiving PoseArray goals.
        
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_subscription_;

        
        // Publisher for the initial pose.
        
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tbot_initial_pose_pub_;

        
        // Flag indicating whether there are active goals. 
        
        rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr client_;

        
        // Flag indicating whether there are active goals.
        
        bool have_goals_;

        //Poses representing waypoints.
        geometry_msgs::msg::Pose wp1_;
        geometry_msgs::msg::Pose wp2_;
        geometry_msgs::msg::Pose wp3_;
        geometry_msgs::msg::Pose wp4_;
        geometry_msgs::msg::Pose wp5_;

        //Vector of PoseStamped representing waypoints.
        
        std::vector<geometry_msgs::msg::PoseStamped> wp_;

    };// class TurtleWaypoints
}//namespace turtle
