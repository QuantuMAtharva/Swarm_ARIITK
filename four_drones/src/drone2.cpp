#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "drone2");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);



//   ROS_INFO("Started hovering");
//   std_srvs::Empty srv;
//   bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
//   unsigned int i = 0;



//   // Trying to unpause Gazebo for 20 seconds.
//   while (i <= 20 && !unpaused) {
//     ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//     unpaused = ros::service::call("/gazebo/unpause_physics", srv);
//     ++i;
//   }
//   if (!unpaused) {
//     ROS_FATAL("Could not wake up Gazebo.");
//     return -1;
//   } else {
//     ROS_INFO("Unpaused the Gazebo simulation.");
//   }
  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();



  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  float x_coord,y_coord,z_coord,yaw_des;
  nh.getParam("x_coord",x_coord);
  nh.getParam("y_coord",y_coord);
  nh.getParam("z_coord",z_coord);
  nh.getParam("yaw_des",yaw_des);

  Eigen::Vector3d desired_position(x_coord, y_coord, z_coord);
  double desired_yaw = yaw_des;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), desired_position.x(),
           desired_position.y(), desired_position.z());
  trajectory_pub.publish(trajectory_msg);








  ros::spinOnce();
  ros::shutdown();

  return 0;
}
