#include "ros/ros.h"
#include <ros/master.h>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <boost/thread.hpp>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit_msgs/MoveGroupActionFeedback.h>
#include <moveit/move_group_interface/move_group.h>

#include <actionlib_msgs/GoalStatus.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>



using namespace std;
using namespace Eigen;
using namespace cv;



class jakaRobot
{
public:  
    //A function whose formal parameter is a reference to a floating-point array 
    int getjoints(double (&arr)[6]);
    int setjoints(double (&arr)[6]);

    tf::StampedTransform getposition(std::string frame1,std::string frame2);
    int go(tf::Transform target, double velocity); //tf--namespace

    jakaRobot();
    ~jakaRobot();

private:
    //MoveGroup class, to realize most operations of a robot in Moveit.
    //the declaration and definition of the class--http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html#setup
    moveit::planning_interface::MoveGroup move_group;
    //Plan--a struct in MoveGroup
    moveit::planning_interface::MoveGroup::Plan plan;
    moveit::planning_interface::PlanningSceneInterface current_scene;
    moveit_msgs::MoveGroupActionFeedback msg;

    geometry_msgs::Pose tf2pose(const tf::Transform &tf);
};


jakaRobot::jakaRobot()
:move_group("jaka_ur")
{
    //moveit::planning_interface::MoveGroup move_group("manipulator");
    move_group.startStateMonitor();//
    move_group.setWorkspace(-1,-1,0,1,1,1.5);//set the maximun work space for the robot
    move_group.setMaxAccelerationScalingFactor(0.1);//valid value,(0,1]
    move_group.setPlanningTime(5);//
    //ROS_INFO, like printf()
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", move_group.getEndEffectorLink().c_str()); 
}

jakaRobot::~jakaRobot()
{

}


int jakaRobot::getjoints(double (&arr)[6])
{
   std::vector<double> joint_values;
   cout<<endl<<"------------------Current Joints(angle):----------"<<endl;
   const std::vector<std::string> &joint_names = move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName())->getJointModelNames();
   move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joint_values);
  
   for(std::size_t i = 0; i < joint_names.size(); i++)
   {
       //ROS_INFO("Joint %s: %f(rad) , %f(degree)", joint_names[i].c_str(), joint_values[i],joint_values[i]/CV_PI*180);
       arr[i] = joint_values[i];
   }
   for(int i = 0; i < joint_names.size(); i++)
      cout<<arr[i]/CV_PI*180<<" ";
   cout<<endl;
   for(int i = 0; i < joint_names.size(); i++)
      cout<<arr[i]<<" ";
   cout<<endl;
   cout<<"------------------------------------------------------"<<endl;

   return 0;
}

int jakaRobot::setjoints(double (&arr)[6])
{
    std::vector<double> aim_values;
    //double arr[6] ={3.14,-1.65,-1.61,-1.41,1.56,0};
    move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), aim_values);
    for(int i = 0; i < 6; i++)
    {
        aim_values[i] = arr[i];
    }
    cout<<endl<<"------------------Set joints:----------"<<endl;
    for(int i = 0; i < aim_values.size(); i++)
       cout<<aim_values[i]<<'\t';
    cout<<endl;
    for(int i = 0; i < aim_values.size(); i++)
       cout<<aim_values[i]/CV_PI*180.0<<'\t';
    cout<<endl;
    move_group.setJointValueTarget(aim_values);
    move_group.move();
    // //bool success = static_cast<bool>(move_group.move());

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
       ROS_INFO("Move to joints success!");
  
    return 0;
} 


tf::StampedTransform jakaRobot::getposition(string frame1 ,string frame2)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {    
        listener.waitForTransform(frame1, frame2, ros::Time(0),ros::Duration(3.0));
        listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());

    }
   cout<<endl<<"[-------------CURRENT TRANSFORM-------------------]: "<<frame1<<'\t'<<frame2<<endl;
   cout<<"x,y,z: "<<transform.getOrigin().x()<<'\t'<<transform.getOrigin().y()<<'\t'<<transform.getOrigin().z()<<endl;
   cout<<"x,y,z,w: "<<transform.getRotation().getX()<<'\t'<<transform.getRotation().getY()<<'\t'<<transform.getRotation().getZ()<<'\t'<<transform.getRotation().getW()<<endl;
   cout<<"------------------------------------------------------"<<endl;
   
   return transform;
}

geometry_msgs::Pose jakaRobot::tf2pose(const tf::Transform &tf)
{
    geometry_msgs::Pose pose;
    tf::Vector3 position = tf.getOrigin();
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];

    tf::Quaternion q = tf.getRotation();
    pose.orientation.x = q[0];
    pose.orientation.y = q[1];
    pose.orientation.z = q[2];
    pose.orientation.w = q[3];
   
    return pose;
}


int jakaRobot::go(tf::Transform target, double velocity)
{
    geometry_msgs::Pose target_pose;
    //target.getOrigin();
    //move_group.setStartState(move_group.getCurrentState());
    move_group.setMaxVelocityScalingFactor(velocity);
   
    target_pose = tf2pose(target);
    //cout<<"----JAKARobot::go-------"<<endl<<target_pose.position<<target_pose.orientation<<endl;
    move_group.setPoseTarget(target_pose);
    bool success =false;
    int attempt =0;

    while(!success)
    {
        move_group.move();
        attempt =attempt+1;
        ROS_INFO("ATTEMPT:%d",attempt);
        //success = static_cast<bool>(move_group.move());
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
        {
            ROS_INFO("Move success!");
            break;
        }
        
        if(attempt>3)
        {
            ROS_INFO("Move failed!");
            break;
        }         
    }   
    return 0;
}




void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
         
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaka_controller");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    jakaRobot r;


    tf::StampedTransform Transform;   
    tf::Transform t;
    tf::Quaternion q;
    double arr[6];
    
    //初始位姿
    // doubl arr_ini[6] = {-0.780329 ,1.326114,1.894454,0.214817,-2.236208,0.935596}; //(-45.02, 76.23, 108.12, 12,83, -127.77, 53.48)
    // r.setjoints(arr_ini);
    
    q=tf::Quaternion(-0.745, 0.248, 0.256, 0.564);
    t.setRotation(q);
    t.setOrigin(tf::Vector3(-0.1028, 0.3468, 0.5108)); //0.5108
    r.go(t,1);

    Transform = r.getposition("/base_link","/ee_link");    
    r.getjoints(arr);
    ROS_INFO("---Arrive at initial pose---");



    ros::shutdown();
    return 0;
}

