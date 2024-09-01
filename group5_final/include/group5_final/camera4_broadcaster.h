/**
 * @file camera4_broadcaster.h
 * @brief Header File for camera 4 Broadcaster 
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
     * @brief Class for camera  4 Broadcaster
    */
    class CameraFourBroadcaster : public rclcpp::Node
    {
        public:
            /**
             * @brief Constructor For camera  4 Broadcaster 
            */
            CameraFourBroadcaster(std::string node_name) : Node(node_name)
            {
                RCLCPP_INFO(this->get_logger(), "Broadcaster started for camera4");

                // Subscription to camera  4 image topic 
                camera4_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera4/image",rclcpp::SensorDataQoS(),
                std::bind(&CameraFourBroadcaster::camera4_sub_cb , this , std::placeholders::_1));

                // Create a timer for broadcasting camera  4 transforms
                camerafour_broadcast_timer_=this->create_wall_timer(1s,std::bind(&CameraFourBroadcaster::camera4_broadcast_timer_cb,this));

                // Initialize StaticTransformBroadcaster for camera4
                tf_broadcaster_camera4_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

                // Initialize Buffer for camera  4 transforms using dedicated thread
                tf_buffer_camera4_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_buffer_camera4_->setUsingDedicatedThread(true);
            }


        private:
            /**
             * @brief Callback function for Camera 4 Subscription 
             * @param msg The received Camera 4 message.
            */
            void camera4_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);

            
            /**
             * @brief Timer Callback for Broadcasting Camera 4 Transformation
            */
            void camera4_broadcast_timer_cb();
            
            // Timer for camera  4 Transform
            rclcpp::TimerBase::SharedPtr camerafour_broadcast_timer_;
            
            // Subscription for camera  4 Image
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera4_subscription_;

            
            // Buffer for camera  4 Transform  
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_camera4_;

            
            // Static Transform Broadcaster for camera  4 
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_camera4_;

            
            //  X- Coordinate for camera  4 transform
            float camerafour_x_; 

            
            //  Y- Coordinate for camera  4 transform
            float camerafour_y_; 

            
            //  Z- Coordinate for camera  4 transform
            float camerafour_z_;  
        
            // X component of Quaternion of camera  4 transform
            float camerafour_q_x_;   

            // Y component of Quaternion of camera  4 transform
            float camerafour_q_y_; 

            // Z component of Quaternion of camera  4 transform
            float camerafour_q_z_;  

            // W component of Quaternion of camera  4 transform
            float camerafour_q_w_; 

            // Variable to store Battery Color 
            std::string battery_color_;

            // Variable to store the Color ID 
            int color_id_;


    };
}