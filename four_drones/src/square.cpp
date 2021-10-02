#include <thread>
#include <chrono>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_disturbance_observer/ObserverState.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <cmath>



geometry_msgs::PointStamped data1,data2,data3,data4;
float x_pos1,y_pos1,z_pos1,x_pos2,y_pos2,z_pos2,x_pos3,y_pos3,z_pos3,x_pos4,y_pos4,z_pos4;

void subcallback1(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  data1=*msg;
  x_pos1=data1.point.x;
  y_pos1=data1.point.y;
  z_pos1=data1.point.z;
//   ROS_INFO("position of firefly 1 ( from  function)= %f %f %f",x_pos1,y_pos1,z_pos1);
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


int main(int argc, char** argv){

  ros::init(argc, argv, "square");
  ros::NodeHandle nh;
  ros::Subscriber pos_sub1 = nh.subscribe<geometry_msgs::PointStamped>
          ("/firefly1/ground_truth/position",100,subcallback1);
      ros::Subscriber pos_sub2 = nh.subscribe<geometry_msgs::PointStamped>
          ("/firefly2/ground_truth/position",100,subcallback2);
      ros::Subscriber pos_sub3 = nh.subscribe<geometry_msgs::PointStamped>
          ("/firefly3/ground_truth/position",100,subcallback3);
      ros::Subscriber pos_sub4 = nh.subscribe<geometry_msgs::PointStamped>
          ("/firefly4/ground_truth/position",100,subcallback4);

      ros::Publisher pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>
          ("/firefly1/command/pose",100);
      ros::Publisher pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
          ("/firefly2/command/pose",100);
      ros::Publisher pos_pub3 = nh.advertise<geometry_msgs::PoseStamped>
          ("/firefly3/command/pose",100);
      ros::Publisher pos_pub4 = nh.advertise<geometry_msgs::PoseStamped>
          ("/firefly4/command/pose",100);
      
      float radius;
      nh.getParam("radius",radius);

  //ROS_INFO("position of firefly 1= %f %f %f",x_pos1,y_pos1,z_pos1);
  float side = 10;
  float x1,x2,x3,x4,y1,y2,y3,y4,z=5;
  float x[4],y[4];
  ros::Rate looprate(100);

  x[0]=side;  y[0]=side;
  x[1]=-side; y[1]=side;
  x[2]=-side; y[2]=-side;
  x[3]=side;  y[3]=-side;

  geometry_msgs::PoseStamped trajectory_msg1,trajectory_msg2,trajectory_msg3,trajectory_msg4;

  int i=0;

  while(ros::ok() && i!=4) // When i!=4 is added, the moving drone leaves the square path at a moment and joins the formation
  {
    ros::spinOnce();
    //ROS_INFO("position of firefly 1= %f %f %f",x_pos1,y_pos1,z_pos1);
    for (i=0;i<4;i++)
    {
      for (int j=1;j<5;j++)
      {
        trajectory_msg1.pose.position.x=x[i]+radius;
        trajectory_msg1.pose.position.y=y[i]+radius;
        trajectory_msg1.pose.position.z=z;

        trajectory_msg2.pose.position.x=x[i]-radius;
        trajectory_msg2.pose.position.y=y[i]-radius;
        trajectory_msg2.pose.position.z=z;

        trajectory_msg3.pose.position.x=x[i]-radius;
        trajectory_msg3.pose.position.y=y[i]+radius;
        trajectory_msg3.pose.position.z=z;

        trajectory_msg4.pose.position.x=x[i]+radius;
        trajectory_msg4.pose.position.y=y[i]-radius;
        trajectory_msg4.pose.position.z=z;
      }

      // pos_pub1.publish(trajectory_msg1);

      // float r1 = sqrt(pow(x_pos1-(x[i]+radius),2)+pow(y_pos1-(y[i]+radius),2)+pow(z_pos1-z,2));
      // float r2 = sqrt(pow(x_pos2-(x[i]-radius),2)+pow(y_pos2-(y[i]-radius),2)+pow(z_pos2-z,2));
      float r3 = sqrt(pow(x_pos3-(x[i]-radius),2)+pow(y_pos3-(y[i]+radius),2)+pow(z_pos3-z,2));
      // float r4 = sqrt(pow(x_pos4-(x[i]+radius),2)+pow(y_pos4-(y[i]-radius),2)+pow(z_pos4-z,2));
      
      while(r3 > 0.5)
      {       
        ros::spinOnce();
        // pos_pub1.publish(trajectory_msg1);
        // pos_pub2.publish(trajectory_msg2);
        pos_pub3.publish(trajectory_msg3);
        // pos_pub4.publish(trajectory_msg4);

        // r1 = sqrt(pow(x_pos1-(x[i]+radius),2)+pow(y_pos1-(y[i]+radius),2)+pow(z_pos1-z,2));
        // r2 = sqrt(pow(x_pos2-(x[i]-radius),2)+pow(y_pos2-(y[i]-radius),2)+pow(z_pos2-z,2));
        r3 = sqrt(pow(x_pos3-(x[i]-radius),2)+pow(y_pos3-(y[i]+radius),2)+pow(z_pos3-z,2));
        // r4 = sqrt(pow(x_pos4-(x[i]+radius),2)+pow(y_pos4-(y[i]-radius),2)+pow(z_pos4-z,2));
      
      }
    }
    looprate.sleep();
  }

}