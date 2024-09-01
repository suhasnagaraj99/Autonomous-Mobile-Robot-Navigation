/**
 * @file camera5_broadcaster.h
 * @brief Header File for camera 5 Broadcaster 
*/
#pragma once
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>


using namespace std::chrono_literals;


namespace turtle{

    /**
     * @brief Class for camera  5 Broadcaster
    */
    class CameraFiveBroadcaster : public rclcpp::Node
    {
        public:
            /**
             * @brief Constructor For camera  5 Broadcaster 
            */

            CameraFiveBroadcaster(std::string node_name) : Node(node_name)
            {
                RCLCPP_INFO(this->get_logger(), "Broadcaster started for camera5");

                // Subscription to camera  5 image topic 
                camera5_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera5/image",rclcpp::SensorDataQoS(),
                std::bind(&CameraFiveBroadcaster::camera5_sub_cb , this , std::placeholders::_1));

                // Create a timer for broadcasting camera  5 transforms
                camerafive_broadcast_timer_=this->create_wall_timer(1s,std::bind(&CameraFiveBroadcaster::camera5_broadcast_timer_cb,this));

                // Initialize StaticTransformBroadcaster for camera5
                tf_broadcaster_camera5_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

                // Initialize Buffer for camera  5 transforms using dedicated thread
                tf_buffer_camera5_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_buffer_camera5_->setUsingDedicatedThread(true);
            }


        private:
            
            /**
             * @brief Callback function for Camera 5 Subscription 
             * @param msg The received Camera 5 message.
            */
            void camera5_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);

            
            /**
             * @brief Timer Callback for Broadcasting Camera 5 Transformation
            */
            void camera5_broadcast_timer_cb();

           
            // Timer for camera  5 Transform
            rclcpp::TimerBase::SharedPtr camerafive_broadcast_timer_;
            
            // Subscription for camera  5 Image
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera5_subscription_;

            // Buffer for camera  5 Transform
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_camera5_;

            
            // Static Transform Broadcaster for camera  5  
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_camera5_;

            
            //  X- Coordinate for camera  5 transform
            float camerafive_x_; 
            
            //  Y- Coordinate for camera  5 transform            
            float camerafive_y_; 
            
            //  Z- Coordinate for camera  5 transform
            float camerafive_z_;  
        
            // X component of Quaternion of camera  5 transform
            float camerafive_q_x_;   

            // Y component of Quaternion of camera  5 transform
            float camerafive_q_y_; 

            // Z component of Quaternion of camera  5 transform
            float camerafive_q_z_;  

            // W component of Quaternion of camera  5 transform
            float camerafive_q_w_; 

            // Variable to store Battery Color 
            std::string battery_color_;

            // Variable to store the Color ID 
            int color_id_;


    };
}