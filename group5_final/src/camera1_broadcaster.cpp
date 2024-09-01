#include "camera1_broadcaster.h"

using namespace std::chrono_literals;


void turtle::CameraOneBroadcaster::camera1_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    
  
    // Extract battery pose information from the message
    cameraone_x_=msg.part_poses[0].pose.position.x;
    cameraone_y_=msg.part_poses[0].pose.position.y;
    cameraone_z_=msg.part_poses[0].pose.position.z;
    cameraone_q_x_=msg.part_poses[0].pose.orientation.x;
    cameraone_q_y_=msg.part_poses[0].pose.orientation.y;
    cameraone_q_z_=msg.part_poses[0].pose.orientation.z;
    cameraone_q_w_=msg.part_poses[0].pose.orientation.w;

    color_id_=msg.part_poses[0].part.color;

    // Assiging frame name based on battery color
    if (color_id_==mage_msgs::msg::Part::RED){
        battery_color_="red_battery_frame";
    }
    if (color_id_==mage_msgs::msg::Part::BLUE){
        battery_color_="blue_battery_frame";
    }
    if (color_id_==mage_msgs::msg::Part::GREEN){
        battery_color_="green_battery_frame";
    }
    if (color_id_==mage_msgs::msg::Part::ORANGE){
        battery_color_="orange_battery_frame";
    }
    if (color_id_==mage_msgs::msg::Part::PURPLE){
        battery_color_="purple_battery_frame";
    }

    // Stop/Reset Subsciption

}


void turtle::CameraOneBroadcaster::camera1_broadcast_timer_cb(){

    if (battery_color_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Battery color not set. Not broadcasting transform.");
        return;
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Broadcasting transform for %s", battery_color_.c_str());
        camera1_subscription_.reset();
    }

    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    
    // Set header information for the transform
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "camera1_frame";
    dynamic_transform_stamped.child_frame_id = battery_color_;

    // Set translation and rotation values for the transform
    dynamic_transform_stamped.transform.translation.x = cameraone_x_;
    dynamic_transform_stamped.transform.translation.y = cameraone_y_;
    dynamic_transform_stamped.transform.translation.z = cameraone_z_;
    dynamic_transform_stamped.transform.rotation.x = cameraone_q_x_;
    dynamic_transform_stamped.transform.rotation.y = cameraone_q_y_;
    dynamic_transform_stamped.transform.rotation.z = cameraone_q_z_;
    dynamic_transform_stamped.transform.rotation.w = cameraone_q_w_;
   
    // Send the transform
    tf_broadcaster_camera1_->sendTransform(dynamic_transform_stamped);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtle::CameraOneBroadcaster>("cameraone_broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
