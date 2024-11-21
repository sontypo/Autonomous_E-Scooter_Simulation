#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <escooter_dynamics/dynamics.h>
#include <escooter_dynamics/control.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "esooter_dynamics");
    ros::NodeHandle nh;

    // Initialize instances of Dynamics and Control
    Dynamics dynamics;
    Control control;

    dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_);
    control.publishOdometry(control.distance_traveled_, control.delta_);
    dynamics.balancingController(control.body_frame_, control.Vx_);
    dynamics.updateDynamics(dynamics.theta_, &control.joints_, dynamics.tau_, control.Vx_);
    control.publishJointCommand(control.joints_);

    ros::AsyncSpinner spinner( 2 ); // Using 2 threads
    spinner.start();

    ros::Rate loop_rate(control.MASTER_LOOP_RATE);
    // ros::Rate loop_rate(100);

    while (ros::ok()) 
    {
        dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_);
        control.publishOdometry(control.distance_traveled_, control.delta_);
        dynamics.balancingController(control.body_frame_, control.Vx_);
        dynamics.updateDynamics(dynamics.theta_, &control.joints_, dynamics.tau_, control.Vx_);
        control.publishJointCommand(control.joints_);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
