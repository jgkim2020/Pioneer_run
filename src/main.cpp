#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <string>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define PI 3.141592

struct Gains{
    double P_gain;
    double I_gain;
    double D_gain;
};

double angular_diff (double theta1, double theta2){


    double theta_diff = theta1 - theta2;

    if(theta_diff > PI)
        theta_diff -= 2*PI;
    else if(theta_diff < -PI)
        theta_diff+= 2*PI;

    return theta_diff;
}



class Pioneer{

public:
    Pioneer();
    geometry_msgs::PoseStamped cur_pose;
    geometry_msgs::Point prev_goal;
    std::string world_frame_id;
    std::string base_frame_id; //mobile
    tf::StampedTransform w2b;
    std::string control_input_topic;
    std::string goal_topic;
    geometry_msgs::Point target_pos_output;
    Gains gains;
    Gains angular_gains;
    double Integral_error;
    ros::Time integral_ckp;
    bool goal_recieved;
    bool isControlled;
    ros::Publisher twist_pub;
    ros::Publisher target_pos_pub;
//    ros::Subscriber pose_sub;
    ros::Subscriber goal_sub;
    ros::NodeHandle nh;
    tf::TransformListener listener;

    double prev_yaw;
    double prev_theta_star;


//    void pose_callback(const geometry_msgs::PoseStampedConstPtr& );
    void goal_callback(const geometry_msgs::PointConstPtr &);
    bool control_input(geometry_msgs::Point,geometry_msgs::Twist&);
    void getTF();
};


Pioneer::Pioneer(): nh("~") {

    nh.getParam("world_frame_id",world_frame_id);
    nh.getParam("base_frame_id",base_frame_id);

    nh.getParam("linear_gain/P",gains.P_gain);
    nh.getParam("linear_gain/I",gains.I_gain);
    nh.getParam("linear_gain/D",gains.D_gain);
    nh.getParam("angular_gain",angular_gains.P_gain);
    nh.getParam("angular_tangential_gain",angular_gains.D_gain);


    nh.getParam("control_input_topic",control_input_topic);
    nh.getParam("goal_topic",goal_topic);

    Integral_error = 0;
    goal_recieved = false;
    isControlled = false;
//    pose_sub = nh.subscribe(pose_topic_name,2,&Pioneer::pose_callback,this);
    goal_sub = nh.subscribe(goal_topic,2,&Pioneer::goal_callback,this);
    twist_pub = nh.advertise<geometry_msgs::Twist>(control_input_topic,2);
    target_pos_pub = nh.advertise<geometry_msgs::Point>("target_position",2);


}



//
//void Pioneer::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
//    cur_pose = *msg;
//}


void Pioneer::getTF() {
    try {

        listener.waitForTransform(world_frame_id, base_frame_id, ros::Time(0), ros::Duration(2.0));
        listener.lookupTransform(world_frame_id, base_frame_id,ros::Time(0),w2b);
        tf::Vector3 position = w2b.getOrigin();
        tf::Quaternion q =w2b.getRotation();
        cur_pose.pose.position.x = position.x();
        cur_pose.pose.position.y = position.y();
        cur_pose.pose.position.z = position.z();

        cur_pose.pose.orientation.x = q.x();
        cur_pose.pose.orientation.y = q.y();
        cur_pose.pose.orientation.z = q.z();
        cur_pose.pose.orientation.w = q.w();

        geometry_msgs::Point cur_pos;
        cur_pos.x = position.x();
        cur_pos.y = position.y();
        cur_pos.z = position.z();
        target_pos_pub.publish(cur_pos);
    }
    catch (tf::TransformException ex){

               ROS_ERROR("%s",ex.what());
               ros::Duration(1.0).sleep();
    }
}

void Pioneer::goal_callback(const geometry_msgs::PointConstPtr & msg) {
    geometry_msgs::Point cur_goal = *msg;
    geometry_msgs::Twist control_input;
    bool insanity =this->control_input(cur_goal,control_input);
    this->twist_pub.publish(control_input);
}

bool Pioneer::control_input(geometry_msgs::Point goal, geometry_msgs::Twist& insert_input) {

    bool insanity;
    double error = sqrt(pow(goal.x - cur_pose.pose.position.x, 2) + pow(goal.y - cur_pose.pose.position.y, 2)) -0.3;
    double dist = sqrt(pow(goal.x - cur_pose.pose.position.x, 2) + pow(goal.y - cur_pose.pose.position.y, 2));
    double tangential = 0;


    if(isControlled) {
        ros::Duration dt = ros::Time::now() - integral_ckp;
        integral_ckp = ros::Time::now();
        Integral_error += error * dt.toSec();
        tangential = atan2(goal.y - prev_goal.y, goal.x - prev_goal.x);

    }else{
        integral_ckp = ros::Time::now();
    }

    geometry_msgs::Twist input;

    printf(" dist : %f error: %f control input:   [%f,%f]\n",dist,error,gains.P_gain * error, gains.I_gain * Integral_error);

    input.linear.x = gains.P_gain * error
                     + gains.I_gain * Integral_error;
    double theta_star = atan2(goal.y - cur_pose.pose.position.y, goal.x - cur_pose.pose.position.x ) ;



    printf(" theta_star = %f/ (y,x) [%f,%f]\n",theta_star,goal.y - cur_pose.pose.position.y,goal.x - cur_pose.pose.position.x);


    tf::Quaternion q(cur_pose.pose.orientation.x,
                     cur_pose.pose.orientation.y,
                     cur_pose.pose.orientation.z,
                     cur_pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);

    printf(" angular diff: %f\n",angular_diff( theta_star , yaw ));
    if(abs(angular_diff( theta_star , yaw )) > 1 )
        insanity = true;
    else
        insanity = false;
    input.angular.z = angular_gains.P_gain * angular_diff( theta_star , yaw ) + angular_gains.D_gain*angular_diff(tangential,yaw);
    insert_input = input;

    prev_theta_star = theta_star;
    prev_yaw = yaw;
    prev_goal = goal;
    isControlled = true;
    return insanity;
}




int main(int argc,char **argv) {
    ros::init(argc,argv,"pioneer_controller");
    Pioneer pioneer;
    ros::Rate loop_rate(100);

    while(ros::ok()){

        pioneer.getTF();
        ros::spinOnce();
        loop_rate.sleep();

    }


}