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



geometry_msgs::PointStamped data1;
float x_pos1,y_pos1,z_pos1;

void subcallback1(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  data1=*msg;
  x_pos1=data1.point.x;
  y_pos1=data1.point.y;
  z_pos1=data1.point.z;
//   ROS_INFO("position of firefly 1 ( from  function)= %f %f %f",x_pos1,y_pos1,z_pos1);
  return;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "square");
    ros::NodeHandle nh;
    ros::Subscriber pos_sub1 = nh.subscribe<geometry_msgs::PointStamped>
            ("/firefly1/ground_truth/position",100,subcallback1);

    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/firefly1/command/pose",100);
    
    //ROS_INFO("position of firefly 1= %f %f %f",x_pos1,y_pos1,z_pos1);
    float side = 10;
    float x1,x2,x3,x4,y1,y2,y3,y4,z=5;
    float x[4],y[4];
    ros::Rate looprate(100);

    x[0]=side;  y[0]=side;
    x[1]=-side; y[1]=side;
    x[2]=-side; y[2]=-side;
    x[3]=side;  y[3]=-side;

    geometry_msgs::PoseStamped trajectory_msg;

    while(ros::ok())
    {
        ros::spinOnce();
        //ROS_INFO("position of firefly 1= %f %f %f",x_pos1,y_pos1,z_pos1);
        for (int i=0;i<4;i++)
        {
                trajectory_msg.pose.position.x=x[i];
                trajectory_msg.pose.position.y=y[i];
                trajectory_msg.pose.position.z=z;

                pos_pub.publish(trajectory_msg);

                float r = sqrt(pow(x_pos1-x[i],2)+pow(y_pos1-y[i],2)+pow(z_pos1-z,2));
                
                while(r > 0.5)
                {       ros::spinOnce();
                        // ROS_INFO("distance from target -> firefly 1= %f",r);
                        pos_pub.publish(trajectory_msg);
                        r = sqrt(pow(x_pos1-x[i],2)+pow(y_pos1-y[i],2)+pow(z_pos1-z,2));
                        // looprate.sleep();
                }
        }
        looprate.sleep();
    }

        


}