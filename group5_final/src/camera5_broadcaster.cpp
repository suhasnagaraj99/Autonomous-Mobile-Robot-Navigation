#include "camera5_broadcaster.h"

using namespace std::chrono_literals;


void turtle::CameraFiveBroadcaster::camera5_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){

    // Extract battery pose information from the message
    camerafive_x_=msg.part_poses[0].pose.position.x;
    camerafive_y_=msg.part_poses[0].pose.position.y;
    camerafive_z_=msg.part_poses[0].pose.position.z;
    camerafive_q_x_=msg.part_poses[0].pose.orientation.x;
    camerafive_q_y_=msg.part_poses[0].pose.orientation.y;
    camerafive_q_z_=msg.part_poses[0].pose.orientation.z;
    camerafive_q_w_=msg.part_poses[0].pose.orientation.w;

    color_id_=msg.part_poses[0].part.color;

    //Assiging frame name based on battery color
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


}

void turtle::CameraFiveBroadcaster::camera5_broadcast_timer_cb(){

    if (battery_color_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Battery color not set. Not broadcasting transform.");
        return;
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Broadcasting transform for %s", battery_color_.c_str());
        camera5_subscription_.reset();
    }

    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    // Set header information for the transform
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "camera5_frame";
    dynamic_transform_stamped.child_frame_id = battery_color_;

    // Set translation and rotation values for the transform
    dynamic_transform_stamped.transform.translation.x = camerafive_x_;
    dynamic_transform_stamped.transform.translation.y = camerafive_y_;
    dynamic_transform_stamped.transform.translation.z = camerafive_z_;
    dynamic_transform_stamped.transform.rotation.x = camerafive_q_x_;
    dynamic_transform_stamped.transform.rotation.y = camerafive_q_y_;
    dynamic_transform_stamped.transform.rotation.z = camerafive_q_z_;
    dynamic_transform_stamped.transform.rotation.w = camerafive_q_w_;
    // Send the transform
    tf_broadcaster_camera5_->sendTransform(dynamic_transform_stamped);
}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Create an instance of the BatteryBroadcaster class
  auto node = std::make_shared<turtle::CameraFiveBroadcaster>("camerafive_broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
