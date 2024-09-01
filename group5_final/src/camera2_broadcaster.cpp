#include "camera2_broadcaster.h"

using namespace std::chrono_literals;


void turtle::CameraTwoBroadcaster::camera2_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){

    // Extract battery pose information from the message
    cameratwo_x_=msg.part_poses[0].pose.position.x;
    cameratwo_y_=msg.part_poses[0].pose.position.y;
    cameratwo_z_=msg.part_poses[0].pose.position.z;
    cameratwo_q_x_=msg.part_poses[0].pose.orientation.x;
    cameratwo_q_y_=msg.part_poses[0].pose.orientation.y;
    cameratwo_q_z_=msg.part_poses[0].pose.orientation.z;
    cameratwo_q_w_=msg.part_poses[0].pose.orientation.w;

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

//     camera2_subscription_.reset();
}

void turtle::CameraTwoBroadcaster::camera2_broadcast_timer_cb(){

    if (battery_color_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Battery color not set. Not broadcasting transform.");
        return;
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Broadcasting transform for %s", battery_color_.c_str());
        camera2_subscription_.reset();
    }

    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    // Set header information for the transform
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "camera2_frame";
    dynamic_transform_stamped.child_frame_id = battery_color_;

    // Set translation and rotation values for the transform
    dynamic_transform_stamped.transform.translation.x = cameratwo_x_;
    dynamic_transform_stamped.transform.translation.y = cameratwo_y_;
    dynamic_transform_stamped.transform.translation.z = cameratwo_z_;
    dynamic_transform_stamped.transform.rotation.x = cameratwo_q_x_;
    dynamic_transform_stamped.transform.rotation.y = cameratwo_q_y_;
    dynamic_transform_stamped.transform.rotation.z = cameratwo_q_z_;
    dynamic_transform_stamped.transform.rotation.w = cameratwo_q_w_;
    // Send the transform
    tf_broadcaster_camera2_->sendTransform(dynamic_transform_stamped);
}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Create an instance of the BatteryBroadcaster class
  auto node = std::make_shared<turtle::CameraTwoBroadcaster>("cameratwo_broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
