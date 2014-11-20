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
    double x_t, y_t, theta_t;
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
        encoder_subscriber = n.subscribe("/kobuki/encoders", 1, &PoseEstimation::encoderCallback,this);
        //encoder_subscriber = n.subscribe("/arduino/encoders", 1, &PoseEstimation::encoderCallback,this);
        odometry_publisher = n.advertise<ras_arduino_msgs::Odometry>("/arduino/odometry", 1);
    }

    void getInitialPose()
    {
        n.getParam("x0",x_prime);
        n.getParam("y0",y_prime);
        n.getParam("theta0",theta_prime);

    }

    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &enc_msg)
    {
        double enc1=enc_msg->encoder1;
        double enc2=enc_msg->encoder2;
        double delta_enc1=enc_msg->delta_encoder1;
        double delta_enc2=enc_msg->delta_encoder2;
        double sampleTime=0.1;
        // RADIO AND BASE FOR KOBUKI
        double b=0.23;
        double r=0.0352;

        // RADIO AND BASE FOR OUR ROBOT
        //double b=0.21;
        //double r=0.05;
        
        double AngVelLeft=(delta_enc2*(M_PI/180))/sampleTime;
        double AngVelRight=(delta_enc1*(M_PI/180))/sampleTime;
        
        // Pose estimate according to formulas from file of Lab3
        x_t = ((-(r*sin(theta_prime))/2)*AngVelLeft + (-(r*sin(theta_prime))/2)*AngVelRight)*sampleTime;
        y_t = (((r*cos(theta_prime))/2)*AngVelLeft + ((r*cos(theta_prime))/2)*AngVelRight)*sampleTime;
        theta_t = ((-r/b)*AngVelLeft + (r/b)*AngVelRight)*sampleTime;

    }

    void poseUpdate()
    {
    	ras_arduino_msgs::Odometry odom_msg;

        // odometry values should be "double"
        x_prime = x_prime + x_t;
        y_prime = y_prime + y_t;
        theta_prime = theta_prime + theta_t;

        odom_msg.x = x_prime;
        odom_msg.y = y_prime;
        odom_msg.theta = theta_prime;

    	odometry_publisher.publish(odom_msg);
    }

private:

    PoseEstimation *pose_estimation_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation");

    PoseEstimation pose;
    pose.init();
    pose.getInitialPose();
    ros::Rate loop_rate(10.0);

    while(pose.n.ok())
    {
        ros::spinOnce();
        pose.poseUpdate();
        loop_rate.sleep();
    }

    return 0;
}
