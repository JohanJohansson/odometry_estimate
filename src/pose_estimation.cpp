#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/Encoders.h"
#include "ras_arduino_msgs/Odometry.h"


class PoseEstimation
{
public:
	ros::NodeHandle n;
    ros::Subscriber velocity_twist_subscriber;
    ros::Subscriber encoder_subscriber;
    ros::Publisher odometry_publisher;
    double x_t, y_t, theta_t_unbounded, theta_t, theta_prime_unbound;
    double x_prime, y_prime, theta_prime;

    PoseEstimation()
    {
        n = ros::NodeHandle("~");
        pose_estimation_ = NULL;
    }

    ~PoseEstimation()
    {
        delete pose_estimation_;
    }

    void init()
    {
        pose_estimation_ = new PoseEstimation();
        // *** USE KOBUKI ENCODERS
        //encoder_subscriber = n.subscribe("/kobuki/encoders", 1, &PoseEstimation::encoderCallback,this);
        // *** USE OUR ROBOT ENCODERS
        encoder_subscriber = n.subscribe("/arduino/encoders", 1, &PoseEstimation::encoderCallback,this);
        odometry_publisher = n.advertise<ras_arduino_msgs::Odometry>("/arduino/odometry", 1);
    }

    void getInitialPose()
    {
        //n.getParam("x0",x_prime);
        //n.getParam("y0",y_prime);
        //n.getParam("theta0",theta_prime);
        x_prime=0;
        y_prime=0;
        theta_prime=0;

    }

    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &enc_msg)
    {
        double delta_enc1 = enc_msg->delta_encoder1;
        double delta_enc2 = enc_msg->delta_encoder2;
        double sampleTime = 0.05;
        // RADIO AND BASE FOR KOBUKI
        //double b=0.23;
        //double r=0.0352;

        // RADIO AND BASE FOR OUR ROBOT
        double b=0.21;
        double r=0.05;
        if(delta_enc1 - delta_enc2 > 3)
            ROS_INFO("bigger delta enc1");
        if(delta_enc2 - delta_enc1 > 3)
            ROS_INFO("dbigger delta enc2");

        //Check which delta encoder corresponds to right and left
        double AngVelLeft =(delta_enc2 * (M_PI/180))/sampleTime;
        //ROS_INFO("AngVelRight %f", AngVelRight);
        double AngVelRight =(delta_enc1 * (M_PI/180))/sampleTime;
        //ROS_INFO("AngVelLeft %f", AngVelLeft);


        // Pose estimate according to formulas from file of Lab3
        x_t = ((-(r*sin(theta_prime))/2.0)*AngVelLeft + (-(r*sin(theta_prime))/2.0)*AngVelRight)*sampleTime;
        y_t = (((r*cos(theta_prime))/2.0)*AngVelLeft + ((r*cos(theta_prime))/2.0)*AngVelRight)*sampleTime;

        theta_t_unbounded = ((-r/b)*AngVelLeft + (r/b)*AngVelRight)*sampleTime;
        theta_t = angleBoundaries(theta_t_unbounded);

    }

    void poseUpdate()
    {
    	ras_arduino_msgs::Odometry odom_msg;

        /*
          if(state == Turn_Left)
            theta_t = +pi/2
            x_t, y_t should not change while turning
          if(state == Turn_Right)
            theta_t = -pi/2
            x_t, y_t should not change while turning

        */

        // odometry values should be "double"
        x_prime = x_prime + x_t;
        y_prime = y_prime + y_t;
        theta_prime = theta_prime + theta_t;
        theta_prime = angleBoundaries(theta_prime);
        //ROS_INFO("theta_prime %f", theta_prime);
        ROS_INFO("x_prime %f", x_prime);
        ROS_INFO("y_prime %f", y_prime);
        ROS_INFO("theta_prime %f", theta_prime);
        //Publish message
        odom_msg.x = x_prime;
        odom_msg.y = y_prime;
        odom_msg.theta = theta_prime;


    	odometry_publisher.publish(odom_msg);
    }

private:

    PoseEstimation *pose_estimation_;

    double angleBoundaries(double theta)
    {
        if (theta > 0)
            theta = fmod(theta + M_PI, 2.0 * M_PI) - M_PI;
        else
            theta = fmod(theta - M_PI, 2.0 * M_PI) + M_PI;
        return theta;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation");

    PoseEstimation pose;
    pose.init();
    pose.getInitialPose();
    ros::Rate loop_rate(20.0);

    while(pose.n.ok())
    {
        ros::spinOnce();
        pose.poseUpdate();
        loop_rate.sleep();
    }

    return 0;
}
