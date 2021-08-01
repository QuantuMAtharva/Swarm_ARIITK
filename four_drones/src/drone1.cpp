#include <thread>
#include <chrono>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <cmath>

geometry_msgs::PointStamped data1,data2,data3,data4;
// nav_msgs::Odometry data5;
// mav_disturbance_observer::ObserverState data5;

float x_pos1,y_pos1,z_pos1,x_pos2,y_pos2,z_pos2,x_pos3,y_pos3,z_pos3,x_pos4,y_pos4,z_pos4;
float x_vel,y_vel,z_vel;
void subcallback1(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  data1=*msg;
  x_pos1=data1.point.x;
  y_pos1=data1.point.y;
  z_pos1=data1.point.z;
  // ROS_INFO("position of firefly 1 ( from  function)= %f %f %f",x_pos1,y_pos1,z_pos1);
  return;
}
void subcallback2(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  data2=*msg;
  x_pos2=data2.point.x;
  y_pos2=data2.point.y;
  z_pos2=data2.point.z;
  //ROS_INFO("position of firefly 2 ( from  function)= %f %f %f",x_pos2,y_pos2,z_pos2);
  return;
}
void subcallback3(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  data3=*msg;
  x_pos3=data3.point.x;
  y_pos3=data3.point.y;
  z_pos3=data3.point.z;
  //ROS_INFO("position of firefly 3 ( from  function)= %f %f %f",x_pos3,y_pos3,z_pos3);
  return;
}
void subcallback4(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  data4=*msg;
  x_pos4=data4.point.x;
  y_pos4=data4.point.y;
  z_pos4=data4.point.z;
  //ROS_INFO("position of firefly 4 ( from  function)= %f %f %f",x_pos4,y_pos4,z_pos4);
  return;
}
// void sub_odom_callback(const mav_disturbance_observer::ObserverState::ConstPtr &msg)
// {
//   data5=*msg;
//   x_vel=data5.velocity[0];
//   y_vel=data5.velocity[1];
//   z_vel=data5.velocity[2];
//   // x_vel=data5.twist.twist.linear.x;
//   // y_vel=data5.twist.twist.linear.y;
//   // z_vel=data5.twist.twist.linear.z;
//   //ROS_INFO("velocity of firefly 1 (from  function)= %f %f %f",x_vel,y_vel,z_vel);
//   return;
// }


int main(int argc, char** argv) {
  ros::init(argc, argv, "drone1");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  // ros::Publisher trajectory_pub =
  //     nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
  //         mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ros::Subscriber pos_sub1 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly1/ground_truth/position",100,subcallback1);
  ros::Subscriber pos_sub2 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly2/ground_truth/position",100,subcallback2);
  ros::Subscriber pos_sub3 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly3/ground_truth/position",100,subcallback3);
  ros::Subscriber pos_sub4 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly4/ground_truth/position",100,subcallback4);
  
  // ros::Subscriber pos_sub5 = nh.subscribe<mav_disturbance_observer::ObserverState>
  //           ("/firefly1/mav_linear_mpc/KF_observer/observer_state",100,sub_odom_callback);
  // ros::Publisher cmd_vel_pub =nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>
  //           ("/firefly1/command/current_reference",100);
  
  ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/firefly1/command/pose",100);


  ROS_INFO("Started hovering");
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;
  // Trying to unpause Gazebo for 20 seconds.
  while (i <= 20 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }
  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }
  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();



  geometry_msgs::PoseStamped trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();


  // Default desired position and yaw.
  float x_coord,y_coord,z_coord,yaw_des,exp_r,radius,min_dist,arm_length,rate,repel_const;
  nh.getParam("x_coord",x_coord);
  nh.getParam("y_coord",y_coord);
  nh.getParam("z_coord",z_coord);
  nh.getParam("yaw_des",yaw_des);
  nh.getParam("radius",radius);
  nh.getParam("exp_r", exp_r);
  nh.getParam("rate", rate);
  nh.getParam("min_dist", min_dist);
  nh.getParam("arm_length", arm_length);
  nh.getParam("repel_const", repel_const);

  // Eigen::Vector3d desired_position(x_coord+radius, y_coord+radius, z_coord);
  // double desired_yaw = yaw_des;

  // Overwrite defaults if set as node parameters.
  // nh_private.param("x", desired_position.x(), desired_position.x());
  // nh_private.param("y", desired_position.y(), desired_position.y());
  // nh_private.param("z", desired_position.z(), desired_position.z());
  // nh_private.param("yaw", desired_yaw, desired_yaw);

  // mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
  //     desired_position, desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(),x_coord+radius,y_coord+radius, z_coord);
  
  trajectory_msg.pose.position.x=x_coord+radius;
  trajectory_msg.pose.position.y=y_coord+radius;
  trajectory_msg.pose.position.z=z_coord;

  //pos_pub.publish(trajectory_msg);
  ros::spinOnce();

  ros::Rate looprate(rate);


  
  float r12,r13,r14;
  float x_near,y_near,z_near;
  float tar_x,tar_y,tar_z;
  float del_rx,del_ry,del_rz, mod_r;
  trajectory_msgs::MultiDOFJointTrajectory vel_pub;
  geometry_msgs::PoseStamped pose;

  while(ros::ok())
  {
    ros::spinOnce();

    // calculate realtive distance of drones
    r12= pow((pow((x_pos1-x_pos2),2)+pow((y_pos1-y_pos2),2)+pow((z_pos1-z_pos2),2)),0.5);
    r13= pow((pow((x_pos1-x_pos3),2)+pow((y_pos1-y_pos3),2)+pow((z_pos1-z_pos3),2)),0.5);
    r14= pow((pow((x_pos1-x_pos4),2)+pow((y_pos1-y_pos4),2)+pow((z_pos1-z_pos4),2)),0.5);
    //ROS_INFO("R12 = %f, %f, %f",r12, r13, r14);
    //ROS_INFO("velocity of firefly 1 = %f %f %f",x_vel,y_vel,z_vel);



    //command drone to stop at its location in case relative distance is less than threshold
    if (r12<min_dist+arm_length||r13<min_dist+arm_length||r14<min_dist+arm_length)
    {
      // nearest drone position
      if (r12<=r13 && r12<=r14)
      {
        //ROS_INFO("r12 is the smallest");
        x_near=x_pos2;
        y_near=y_pos2;
        z_near=z_pos2;
      }    
      else if (r13 <= r12 && r13 <= r14)
      {
        //ROS_INFO("r13 is the smallest");
        x_near=x_pos3;
        y_near=y_pos3;
        z_near=z_pos3;
      }
      else
      {
        //ROS_INFO("r14 is the smallest");
        x_near=x_pos4;
        y_near=y_pos4;
        z_near=z_pos4;
      }



      // calculate new velocity to avoid collision
      del_rx=x_pos1-x_near;
      del_ry=y_pos1-y_near;
      del_rz=z_pos1-z_near;
      mod_r=pow((pow(del_rx,2)+pow(del_ry,2)+pow(del_rz,2)),0.5);

      tar_x = x_pos1 + repel_const*(del_rx/pow(mod_r,exp_r));
      tar_y = y_pos1 + repel_const*(del_ry/pow(mod_r,exp_r));
      tar_z = z_pos1 + repel_const*(del_rz/pow(mod_r,exp_r));
      

      pose.pose.position.x=tar_x;
      pose.pose.position.y=tar_y;
      pose.pose.position.z=tar_z;

      pos_pub.publish(pose);

      for(int i=0;i<5;i++){
        ros::spinOnce();
        looprate.sleep(); // 5/10= 0.5 sec of wait
      }

      
      pose.pose.position.x=x_coord+radius;
      pose.pose.position.y=y_coord+radius;
      pose.pose.position.z=z_coord;
      pos_pub.publish(pose);
    }


    //ROS_INFO("position of firefly 1= %f %f %f",x_pos1,y_pos1,z_pos1);
    looprate.sleep();
  }


  ros::shutdown();

  return 0;
}


