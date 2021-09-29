#include <thread>
#include <chrono>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_disturbance_observer/ObserverState.h>
#include <cmath>
#include <algorithm>
#include <numeric>

geometry_msgs::PointStamped data1,data2,data3,data4;
// nav_msgs::Odometry data5;
mav_disturbance_observer::ObserverState data5;

float x_pos1,y_pos1,z_pos1,x_pos2,y_pos2,z_pos2,x_pos3,y_pos3,z_pos3,x_pos4,y_pos4,z_pos4;
float x_vel1,y_vel1,z_vel1,x_vel2,y_vel2,z_vel2,x_vel3,y_vel3,z_vel3,x_vel4,y_vel4,z_vel4;

// Callbacks for getting positions of drones

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
void sub_odom_callback1(const mav_disturbance_observer::ObserverState::ConstPtr &msg)
{
  data5=*msg;
  x_vel1=data5.velocity[0];
  y_vel1=data5.velocity[1];
  z_vel1=data5.velocity[2];
  // x_vel=data5.twist.twist.linear.x;
  // y_vel=data5.twist.twist.linear.y;
  // z_vel=data5.twist.twist.linear.z;
  //ROS_INFO("velocity of firefly 1 (from  function)= %f %f %f",x_vel,y_vel,z_vel);
  return;
}
void sub_odom_callback2(const mav_disturbance_observer::ObserverState::ConstPtr &msg)
{
  data5=*msg;
  x_vel2=data5.velocity[0];
  y_vel2=data5.velocity[1];
  z_vel2=data5.velocity[2];
  // x_vel=data5.twist.twist.linear.x;
  // y_vel=data5.twist.twist.linear.y;
  // z_vel=data5.twist.twist.linear.z;
  // ROS_INFO("velocity of firefly 2 (from  function)= %f %f %f",x_vel,y_vel,z_vel);
  return;
}
void sub_odom_callback3(const mav_disturbance_observer::ObserverState::ConstPtr &msg)
{
  data5=*msg;
  x_vel3=data5.velocity[0];
  y_vel3=data5.velocity[1];
  z_vel3=data5.velocity[2];
  // x_vel=data5.twist.twist.linear.x;
  // y_vel=data5.twist.twist.linear.y;
  // z_vel=data5.twist.twist.linear.z;
  //ROS_INFO("velocity of firefly 3 (from  function)= %f %f %f",x_vel,y_vel,z_vel);
  return;
}
void sub_odom_callback4(const mav_disturbance_observer::ObserverState::ConstPtr &msg)
{
  data5=*msg;
  x_vel4=data5.velocity[0];
  y_vel4=data5.velocity[1];
  z_vel4=data5.velocity[2];
  // x_vel=data5.twist.twist.linear.x;
  // y_vel=data5.twist.twist.linear.y;
  // z_vel=data5.twist.twist.linear.z;
  //ROS_INFO("velocity of firefly 4 (from  function)= %f %f %f",x_vel,y_vel,z_vel);
  return;
}

float app_vel(std::vector<float>vec_pos1,std::vector<float>vec_outdrone_pos,std::vector<float>vec_vel1,std::vector<float>vec_outdrone_vel)
{
  float vel1,vel2;
  std::vector<float> vec_del_pos {vec_pos1[0]-vec_outdrone_pos[0] , vec_pos1[1]-vec_outdrone_pos[1] , vec_pos1[2]-vec_outdrone_pos[2]};
  if(vec_del_pos[0]<0.1 && vec_del_pos[1]<0.1 && vec_del_pos[2]<0.1)
  {
    vec_del_pos[0]=0.1;vec_del_pos[1]=0.1;vec_del_pos[2]=0.1;
  }

  vel1 = std::inner_product(vec_vel1.begin(), vec_vel1.end(), vec_del_pos.begin(), 0.0)/(pow(pow(vec_del_pos[0],2)+pow(vec_del_pos[1],2)+pow(vec_del_pos[2],2),0.5));
  vel2 = std::inner_product(vec_outdrone_vel.begin(), vec_outdrone_vel.end(), vec_del_pos.begin(), 0.0)/(pow(pow(vec_del_pos[0],2)+pow(vec_del_pos[1],2)+pow(vec_del_pos[2],2),0.5));
  
  if(vel1-vel2>0)
  return(0);

  return vel1-vel2;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "drone_repel_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");


  // Subscribers and publishers ---------------------------------------------------------------------------

  ros::Subscriber pos_sub1 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly1/ground_truth/position",100,subcallback1);
  ros::Subscriber pos_sub2 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly2/ground_truth/position",100,subcallback2);
  ros::Subscriber pos_sub3 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly3/ground_truth/position",100,subcallback3);
  ros::Subscriber pos_sub4 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly4/ground_truth/position",100,subcallback4);

  ros::Subscriber vel_sub1 = nh.subscribe<mav_disturbance_observer::ObserverState>
            ("/firefly1/mav_linear_mpc/KF_observer/observer_state",100,sub_odom_callback1);
  ros::Subscriber vel_sub2 = nh.subscribe<mav_disturbance_observer::ObserverState>
            ("/firefly2/mav_linear_mpc/KF_observer/observer_state",100,sub_odom_callback2);
  ros::Subscriber vel_sub3 = nh.subscribe<mav_disturbance_observer::ObserverState>
            ("/firefly3/mav_linear_mpc/KF_observer/observer_state",100,sub_odom_callback3);
  ros::Subscriber vel_sub4 = nh.subscribe<mav_disturbance_observer::ObserverState>
            ("/firefly4/mav_linear_mpc/KF_observer/observer_state",100,sub_odom_callback4);
 
  
  // ros::Subscriber pos_sub5 = nh.subscribe<mav_disturbance_observer::ObserverState>
  //           ("/firefly1/mav_linear_mpc/KF_observer/observer_state",100,sub_odom_callback);
  // ros::Publisher cmd_vel_pub =nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>
  //           ("/firefly1/command/current_reference",100);
  
  // Publish position to /command/pose topic of each drone

  ros::Publisher pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>
            ("/firefly1/command/pose",100);
  ros::Publisher pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
            ("/firefly2/command/pose",100);
  ros::Publisher pos_pub3 = nh.advertise<geometry_msgs::PoseStamped>
            ("/firefly3/command/pose",100); 
  ros::Publisher pos_pub4 = nh.advertise<geometry_msgs::PoseStamped>
            ("/firefly4/command/pose",100); 


  // Gazebo things -----------------------------------------------------------------------------------------------

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


  // Get params  -----------------------------------------------------------------------------------------------
  float x_coord,y_coord,z_coord,yaw_des,exp_r,radius,min_dist,arm_length,rate,repel_const,min_dist_vel,vel_repel_const;
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
  nh.getParam("min_dist_vel", min_dist_vel);
  nh.getParam("vel_repel_const", vel_repel_const);


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
  

  // Setting traget locations -----------------------------------------------------------------------------------------------
  
  geometry_msgs::PoseStamped trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  
  trajectory_msg.pose.position.x=x_coord+radius;
  trajectory_msg.pose.position.y=y_coord-+radius;
  trajectory_msg.pose.position.z=z_coord;
  pos_pub1.publish(trajectory_msg);
  
  trajectory_msg.pose.position.x=x_coord-radius;
  trajectory_msg.pose.position.y=y_coord-radius;
  trajectory_msg.pose.position.z=z_coord;
  pos_pub2.publish(trajectory_msg);

  trajectory_msg.pose.position.x=x_coord-radius;
  trajectory_msg.pose.position.y=y_coord+radius;
  trajectory_msg.pose.position.z=z_coord;
  // pos_pub3.publish(trajectory_msg);
  
  trajectory_msg.pose.position.x=x_coord+radius;
  trajectory_msg.pose.position.y=y_coord-radius;
  trajectory_msg.pose.position.z=z_coord;
  pos_pub4.publish(trajectory_msg);

  ros::spinOnce();

  ros::Rate looprate(rate);

// Variable monster -------------------------------------------------------------------------------
  
  float r12,r13,r14,r23,r24,r34;
  float x_near1,y_near1,z_near1,x_near2,y_near2,z_near2,x_near3,y_near3,z_near3,x_near4,y_near4,z_near4;
  float x_near1b,y_near1b,z_near1b,x_near2b,y_near2b,z_near2b,x_near3b,y_near3b,z_near3b,x_near4b,y_near4b,z_near4b;
  
  float vx_near1,vy_near1,vz_near1,vx_near2,vy_near2,vz_near2,vx_near3,vy_near3,vz_near3,vx_near4,vy_near4,vz_near4;
  float vx_near1b, vy_near1b, vz_near1b, vx_near2b, vy_near2b, vz_near2b, vx_near3b, vy_near3b, vz_near3b, vx_near4b, vy_near4b, vz_near4b;
  
  float del_rx1b, del_ry1b, del_rz1b, del_rx2b, del_ry2b, del_rz2b, del_rx3b, del_ry3b, del_rz3b, del_rx4b, del_ry4b, del_rz4b;

  float tar_x1,tar_y1,tar_z1,tar_x2,tar_y2,tar_z2,tar_x3,tar_y3,tar_z3,tar_x4,tar_y4,tar_z4;
  
  float del_rx1,del_ry1,del_rz1,del_rx2,del_ry2,del_rz2,del_rx3,del_ry3,del_rz3,del_rx4,del_ry4,del_rz4,mod_r1,mod_r2,mod_r3,mod_r4;
  
  float del_vx1,del_vy1,del_vz1,del_vx2,del_vy2,del_vz2,del_vx3,del_vy3,del_vz3,del_vx4,del_vy4,del_vz4,mod_v1,mod_v2,mod_v3,mod_v4;
  float del_vx1b, del_vy1b, del_vz1b, del_vx2b, del_vy2b, del_vz2b, del_vx3b, del_vy3b, del_vz3b, del_vx4b, del_vy4b, del_vz4b;
  
  geometry_msgs::PoseStamped pose1,pose2,pose3,pose4;


  while(ros::ok())
  {
    ros::spinOnce();
    // calculate realtive distance of drones -------------------------------------------------------------------------------

    r12= pow((pow((x_pos1-x_pos2),2)+pow((y_pos1-y_pos2),2)+pow((z_pos1-z_pos2),2)),0.5);
    r13= pow((pow((x_pos1-x_pos3),2)+pow((y_pos1-y_pos3),2)+pow((z_pos1-z_pos3),2)),0.5);
    r14= pow((pow((x_pos1-x_pos4),2)+pow((y_pos1-y_pos4),2)+pow((z_pos1-z_pos4),2)),0.5);
    r23= pow((pow((x_pos2-x_pos3),2)+pow((y_pos2-y_pos3),2)+pow((z_pos2-z_pos3),2)),0.5);
    r24= pow((pow((x_pos2-x_pos4),2)+pow((y_pos2-y_pos4),2)+pow((z_pos2-z_pos4),2)),0.5);
    r34= pow((pow((x_pos3-x_pos4),2)+pow((y_pos3-y_pos4),2)+pow((z_pos3-z_pos4),2)),0.5);


  if (r12<(min_dist_vel+arm_length) || r13<(min_dist_vel+arm_length) || r14<(min_dist_vel+arm_length) || r23<(min_dist_vel+arm_length) || r24<(min_dist_vel+arm_length) || r34<(min_dist_vel+arm_length))
    {
      // nearest drone, their relative velocities and relative distances ----------------------------------------------------
      // nearest drone position for drone 1
      if (r12<=r13 && r12<=r14)
      {
        //ROS_INFO("r12 is the smallest");
        x_near1=x_pos2;
        y_near1=y_pos2;
        z_near1=z_pos2;

        vx_near1=x_vel2;
        vy_near1=y_vel2;
        vz_near1=z_vel2;
      }    
      else if (r13 <= r12 && r13 <= r14)
      {
        //ROS_INFO("r13 is the smallest");
        x_near1=x_pos3;
        y_near1=y_pos3;
        z_near1=z_pos3;

        vx_near1=x_vel3;
        vy_near1=y_vel3;
        vz_near1=z_vel3;
      }
      else
      {
        //ROS_INFO("r14 is the smallest");
        x_near1=x_pos4;
        y_near1=y_pos4;
        z_near1=z_pos4;

        vx_near1=x_vel4;
        vy_near1=y_vel4;
        vz_near1=z_vel4;
      }



      // nearest drone position for drone 2
      if (r12<=r23 && r12<=r24)
      {
        //ROS_INFO("r12 is the smallest");
        x_near2=x_pos1;
        y_near2=y_pos1;
        z_near2=z_pos1;

        vx_near2=x_vel1;
        vy_near2=y_vel1;
        vz_near2=z_vel1;
      }    
      else if (r23 <= r12 && r23 <= r24)
      {
        //ROS_INFO("r13 is the smallest");
        x_near2=x_pos3;
        y_near2=y_pos3;
        z_near2=z_pos3;

        vx_near2=x_vel3;
        vy_near2=y_vel3;
        vz_near2=z_vel3;
      }
      else
      {
        //ROS_INFO("r14 is the smallest");
        x_near2=x_pos4;
        y_near2=y_pos4;
        z_near2=z_pos4;

        vx_near2=x_vel4;
        vy_near2=y_vel4;
        vz_near2=z_vel4;
      }




      // nearest drone position for drone 3
      if (r13<=r23 && r13<=r34)
      {
        //ROS_INFO("r12 is the smallest");
        x_near3=x_pos1;
        y_near3=y_pos1;
        z_near3=z_pos1;

        vx_near3=x_vel1;
        vy_near3=y_vel1;
        vz_near3=z_vel1;
      }    
      else if (r23 <= r13 && r23 <= r34)
      {
        //ROS_INFO("r13 is the smallest");
        x_near3=x_pos2;
        y_near3=y_pos2;
        z_near3=z_pos2;

        vx_near3=x_vel2;
        vy_near3=y_vel2;
        vz_near3=z_vel2;
      }
      else
      {
        //ROS_INFO("r14 is the smallest");
        x_near3=x_pos4;
        y_near3=y_pos4;
        z_near3=z_pos4;

        vx_near3=x_vel4;
        vy_near3=y_vel4;
        vz_near3=z_vel4;
      }



     // nearest drone position for drone 4
      if (r14 <= r34 && r14 <= r24)
      {
        //ROS_INFO("r41 is the smallest");
        x_near4=x_pos1;
        y_near4=y_pos1;
        z_near4=z_pos1;

        vx_near4=x_vel1;
        vy_near4=y_vel1;
        vz_near4=z_vel1;
      }    
      else if (r24 <= r14 && r24 <= r34)
      {
        //ROS_INFO("r42 is the smallest");
        x_near4=x_pos2;
        y_near4=y_pos2;
        z_near4=z_pos2;

        vx_near4=x_vel2;
        vy_near4=y_vel2;
        vz_near4=z_vel2;
      }
      else
      {
        //ROS_INFO("r43 is the smallest");
        x_near4=x_pos3;
        y_near4=y_pos3;
        z_near4=z_pos3;

        vx_near4=x_vel3;
        vy_near4=y_vel3;
        vz_near4=z_vel3;
      }


      // rel velocity for 1
      del_vx1=x_vel1-vx_near1;
      del_vy1=y_vel1-vy_near1;
      del_vz1=z_vel1-vz_near1;
      mod_v1=pow((pow(del_vx1,2)+pow(del_vy1,2)+pow(del_vz1,2)),0.5);
    
      // rel distance for 1
      del_rx1=x_pos1-x_near1;
      del_ry1=y_pos1-y_near1;
      del_rz1=z_pos1-z_near1;
      mod_r1=pow((pow(del_rx1,2)+pow(del_ry1,2)+pow(del_rz1,2)),0.5);



      // relative vel for 2
      del_vx2=x_vel2-vx_near2;
      del_vy2=y_vel2-vy_near2;
      del_vz2=z_vel2-vz_near2;
      mod_v2=pow((pow(del_vx2,2)+pow(del_vy2,2)+pow(del_vz2,2)),0.5);

      // rel distance for 2
      del_rx2=x_pos2-x_near2;
      del_ry2=y_pos2-y_near2;
      del_rz2=z_pos2-z_near2;
      mod_r2=pow((pow(del_rx2,2)+pow(del_ry2,2)+pow(del_rz2,2)),0.5);
      


      // relative vel for 3
      del_vx3=x_vel3-vx_near3;
      del_vy3=y_vel3-vy_near3;
      del_vz3=z_vel3-vz_near3;
      mod_v3=pow((pow(del_vx3,2)+pow(del_vy3,2)+pow(del_vz3,2)),0.5);

      // rel distance for 3
      del_rx3=x_pos3-x_near3;
      del_ry3=y_pos3-y_near3;
      del_rz3=z_pos3-z_near3;
      mod_r3=pow((pow(del_rx3,2)+pow(del_ry3,2)+pow(del_rz3,2)),0.5);



      // calculate and publish new velocity for drone 4
      del_vx4=x_vel4-vx_near4;
      del_vy4=y_vel4-vy_near4;
      del_vz4=z_vel4-vz_near4;
      mod_v4=pow((pow(del_vx4,2)+pow(del_vy4,2)+pow(del_vz4,2)),0.5);

      // rel distance for 4
      del_rx4=x_pos4-x_near4;
      del_ry4=y_pos4-y_near4;
      del_rz4=z_pos4-z_near4;
      mod_r4=pow((pow(del_rx4,2)+pow(del_ry4,2)+pow(del_rz4,2)),0.5);


      // Calculation for middle region -------------------------------------------------------------------------------------------

      std::vector<float> d;
      d.clear();
      if(r12>min_dist_vel+arm_length)d.push_back(0);
      else d.push_back(r12);
      
      if(r13>min_dist_vel+arm_length)d.push_back(0);
      else d.push_back(r13);

      if(r14>min_dist_vel+arm_length)d.push_back(0);
      else d.push_back(r14);

      int max_index = max_element(d.begin(),d.end())-d.begin();

      // ROS_INFO("r12:%f r13:%f r14:%f", r12, r13, r14);
      // ROS_INFO("chala ja");
      // ROS_INFO("max_index %f", d[max_index]);

      // int key=1;
      // if(d[max_index]<min_dist+arm_length)key=0;
      // else key=1;

      float outdrone_x,outdrone_y,outdrone_z,outdrone_vx,outdrone_vy,outdrone_vz;
        

        if(max_index==0)
        {
          outdrone_x=x_pos2;
          outdrone_y=y_pos2;
          outdrone_z=z_pos2;
          outdrone_vx=x_vel2;
          outdrone_vy=y_vel2;
          outdrone_vz=z_vel2;
        }
        else if(max_index==1)
        {
          outdrone_x=x_pos3;
          outdrone_y=y_pos3;
          outdrone_z=z_pos3;
          outdrone_vx=x_vel3;
          outdrone_vy=y_vel3;
          outdrone_vz=z_vel3;
        }
        else if(max_index==2)
        {
          outdrone_x=x_pos4;
          outdrone_y=y_pos4;
          outdrone_z=z_pos4;
          outdrone_vx=x_vel4;
          outdrone_vy=y_vel4;
          outdrone_vz=z_vel4;
        }
        else;
        std::vector<float> vec_outdrone_pos {outdrone_x,outdrone_y,outdrone_z};
        std::vector<float> vec_outdrone_vel {outdrone_vx,outdrone_vy,outdrone_vz};
        std::vector<float> vec_pos1 {x_pos1,y_pos1,z_pos1};
        std::vector<float> vec_pos2 {x_pos2,y_pos2,z_pos2};
        std::vector<float> vec_pos3 {x_pos3,y_pos3,z_pos3};
        std::vector<float> vec_pos4 {x_pos4,y_pos4,z_pos4};
        std::vector<float> vec_vel1 {x_vel1,y_vel1,z_vel1};
        std::vector<float> vec_vel2 {x_vel2,y_vel2,z_vel2};
        std::vector<float> vec_vel3 {x_vel3,y_vel3,z_vel3};
        std::vector<float> vec_vel4 {x_vel4,y_vel4,z_vel4};
      // Condition for middle region -----------------------------------------------------------------------------
      // if (r12>min_dist+arm_length || r13>min_dist+arm_length || r14>min_dist+arm_length)
      if (d[max_index]>radius*2*1.732 + 1.5)
      {
        ROS_INFO("middle region");        
        ROS_INFO("imposter drone = %d", max_index+2);

        // tar_x1 = x_pos1 + repel_const*(del_rx1/pow(mod_r1,exp_r));        
        // tar_y1 = y_pos1 + repel_const*(del_ry1/pow(mod_r1,exp_r));
        // tar_x1 = x_pos1 - 100*mod_v1*(del_vy1);
        // tar_y1 = y_pos1 - 100*mod_v1*(del_vx1);
        // tar_z1 = z_pos1 + repel_const*(del_rz1/pow(mod_r1,exp_r));        
        
        float app_v1= app_vel(vec_pos1,vec_outdrone_pos,vec_vel1,vec_outdrone_vel);
        float del_r1 = pow((pow((x_pos1-outdrone_x),2)+ pow((y_pos1-outdrone_y),2) + pow((z_pos1-outdrone_z),2)),0.5);
        ROS_INFO("del_r1 from imposter = %f",del_r1);
        if (del_r1<1) del_r1 = 1;

        tar_z1 = z_pos1 - vel_repel_const*app_v1/del_r1;
        ROS_INFO("tar_z for 1 = %f",tar_z1);
        ROS_INFO("app vel = %f",app_v1);
        pose1.pose.position.x=x_pos1;
        pose1.pose.position.y=y_pos1;
        pose1.pose.position.z=tar_z1;


        // tar_x2 = x_pos2 + repel_const*(del_rx2/pow(mod_r2,exp_r));
        // tar_y2 = y_pos2 + repel_const*(del_ry2/pow(mod_r2,exp_r));
        // tar_x2 = x_pos2 - 100*mod_v2*(del_vy2);
        // tar_y2 = y_pos2 - 100*mod_v2*(del_vx2);
        // tar_z2 = z_pos2 + repel_const*(del_rz2/pow(mod_r2,exp_r));
        float app_v2= app_vel(vec_pos2,vec_outdrone_pos,vec_vel2,vec_outdrone_vel);
        float del_r2 = pow((pow((x_pos2-outdrone_x),2)+ pow((y_pos2-outdrone_y),2) + pow((z_pos2-outdrone_z),2)),0.5);

        if (del_r2<1) del_r2 = 1;

        tar_z2 = z_pos2 - vel_repel_const*app_v2/del_r2; 
        pose2.pose.position.x=x_pos2;
        pose2.pose.position.y=y_pos2;
        pose2.pose.position.z=tar_z2;


        // tar_x3 = x_pos3 + repel_const*(del_rx3/pow(mod_r3,exp_r));
        // tar_y3 = y_pos3 + repel_const*(del_ry3/pow(mod_r3,exp_r));
        // tar_x3 = x_pos3 - 100*mod_v3*(del_vy3);
        // tar_y3 = y_pos3 - 100*mod_v3*(del_vx3);
        // tar_z3 = z_pos3 + repel_const*(del_rz3/pow(mod_r3,exp_r));
        float app_v3= app_vel(vec_pos3,vec_outdrone_pos,vec_vel3,vec_outdrone_vel);
        float del_r3 = pow((pow((x_pos3-outdrone_x),2)+ pow((y_pos3-outdrone_y),2) + pow((z_pos3-outdrone_z),2)),0.5);

        if (del_r3<1) del_r3 = 1;

        tar_z3 = z_pos3 - vel_repel_const*app_v3/del_r3;   
        pose3.pose.position.x=x_pos3;
        pose3.pose.position.y=y_pos3;
        pose3.pose.position.z=tar_z3;

        
        // tar_x4 = x_pos4 + repel_const*(del_rx4/pow(mod_r4,exp_r));
        // tar_y4 = y_pos4 + repel_const*(del_ry4/pow(mod_r4,exp_r));
        // tar_x4 = x_pos4 - 100*mod_v4*(del_vy4);
        // tar_y4 = y_pos4 - 100*mod_v4*(del_vx4);
        // tar_z4 = z_pos4 + repel_const*(del_rz4/pow(mod_r4,exp_r));
        float app_v4= app_vel(vec_pos4,vec_outdrone_pos,vec_vel4,vec_outdrone_vel);
        float del_r4 = pow((pow((x_pos4-outdrone_x),2)+ pow((y_pos4-outdrone_y),2) + pow((z_pos4-outdrone_z),2)),0.5);

        if (del_r4<1) del_r4 = 1;

        tar_z4 = z_pos4 - vel_repel_const*app_v4/del_r4;
        
        pose4.pose.position.x=x_pos4;
        pose4.pose.position.y=y_pos4;
        pose4.pose.position.z=tar_z4;

        // publish new position
        pos_pub1.publish(pose1);
        pos_pub2.publish(pose2);
        pos_pub3.publish(pose3);
        pos_pub4.publish(pose4);

        for(int i=0;i<3;i++)
        {
          ros::spinOnce();
          looprate.sleep(); // 5/10= 0.5 sec of wait
        }

      }

    // Condition for inner region ---------------------------------------------------------------------------------
      if (r12<min_dist+arm_length || r13<min_dist+arm_length || r14<min_dist+arm_length || r23<min_dist+arm_length || r24<min_dist+arm_length || r34<min_dist+arm_length)
      {
        ROS_INFO("inner loop");

        tar_x1 = x_pos1 + repel_const*(del_rx1/pow(mod_r1,exp_r));
        tar_y1 = y_pos1 + repel_const*(del_ry1/pow(mod_r1,exp_r));
        tar_z1 = z_pos1 + repel_const*(del_rz1/pow(mod_r1,exp_r));
        pose1.pose.position.x=tar_x1;
        pose1.pose.position.y=tar_y1;
        pose1.pose.position.z=tar_z1;


        tar_x2 = x_pos2 + repel_const*(del_rx2/pow(mod_r2,exp_r));
        tar_y2 = y_pos2 + repel_const*(del_ry2/pow(mod_r2,exp_r));
        tar_z2 = z_pos2 + repel_const*(del_rz2/pow(mod_r2,exp_r));        
        pose2.pose.position.x=tar_x2;
        pose2.pose.position.y=tar_y2;
        pose2.pose.position.z=tar_z2;


        tar_x3 = x_pos3 + repel_const*(del_rx3/pow(mod_r3,exp_r));
        tar_y3 = y_pos3 + repel_const*(del_ry3/pow(mod_r3,exp_r));
        tar_z3 = z_pos3 + repel_const*(del_rz3/pow(mod_r3,exp_r));        
        pose3.pose.position.x=tar_x3;
        pose3.pose.position.y=tar_y3;
        pose3.pose.position.z=tar_z3;

        tar_x4 = x_pos4 + repel_const*(del_rx4/pow(mod_r4,exp_r));
        tar_y4 = y_pos4 + repel_const*(del_ry4/pow(mod_r4,exp_r));
        tar_z4 = z_pos4 + repel_const*(del_rz4/pow(mod_r4,exp_r));
        
        pose4.pose.position.x=tar_x4;
        pose4.pose.position.y=tar_y4;
        pose4.pose.position.z=tar_z4;

        // publish new position
        pos_pub1.publish(pose1);
        pos_pub2.publish(pose2);
        pos_pub3.publish(pose3);
        pos_pub4.publish(pose4);

        for(int i=0;i<3;i++)
        {
          ros::spinOnce();
          looprate.sleep(); // 5/10= 0.5 sec of wait
        }
        
      }



      // Update the target locations again --------------------------------------------------------------------------
      pose1.pose.position.x=x_coord+radius;
      pose1.pose.position.y=y_coord+radius;
      pose1.pose.position.z=z_coord;

      pose2.pose.position.x=x_coord-radius;
      pose2.pose.position.y=y_coord-radius;
      pose2.pose.position.z=z_coord;

      pose3.pose.position.x=x_coord-radius;
      pose3.pose.position.y=y_coord+radius;
      pose3.pose.position.z=z_coord;

      pose4.pose.position.x=x_coord+radius;
      pose4.pose.position.y=y_coord-radius;
      pose4.pose.position.z=z_coord;

      pos_pub1.publish(pose1);
      pos_pub2.publish(pose2);
      pos_pub3.publish(pose3);
      pos_pub4.publish(pose4);
    }
    

    //ROS_INFO("position of firefly 1= %f %f %f",x_pos1,y_pos1,z_pos1);
    looprate.sleep();
  }


  ros::shutdown();

  return 0;
}