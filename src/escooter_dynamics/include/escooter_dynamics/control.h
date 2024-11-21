#ifndef CONTROL_H_
#define CONTROL_H_

#include <cmath>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <escooter_msgs/Joints.h>
#include <escooter_msgs/Pose.h>
#include <escooter_msgs/RPY.h>

class Control
{
    public:
        Control(void);
        void setScooterState(bool state);
        bool getScooterState(void);
        void setPrevScooterState(bool state);
        bool getPrevScooterState(void);
        void publishJointCommand(const escooter_msgs::Joints &joints);
        void publishOdometry(double distance_traveled, double delta);
        void publishTwist(const geometry_msgs::Twist &com_vel);
        // void pidController();
        int MASTER_LOOP_RATE;   // Master loop rate
        sensor_msgs::JointState joint_state_;
        escooter_msgs::Pose body_frame_;    // Body link rotation
        escooter_msgs::Joints joints_;
        geometry_msgs::Twist com_vel_;
        geometry_msgs::Twist cmd_vel_;
        double distance_traveled_ = 0.0;
        double delta_ = 0.0;
        double prev_delta_ = 0.0;
        double delta_dot_ = 0.0;
        double Vx_, des_delta;
        double Vx_dot_ = 0.0;

    private:
        bool imu_init_stored_; // Auto-levelling
        double imu_roll_lowpass_, imu_pitch_lowpass_, imu_yaw_lowpass_, imu_roll_init_, imu_pitch_init_; // Auto-levelling
        double MAX_BODY_ROLL_COMP, MAX_BODY_PITCH_COMP, COMPENSATE_INCREMENT, COMPENSATE_TO_WITHIN, MAX_STEERING_ANGLE; // Auto-levelling yaml config
        double BODY_MAX_ROLL, BODY_MAX_PITCH, BODY_MAX_YAW; // Mechanical limits
        double VELOCITY_DIVISION, WHEEL_RADIUS, WHEEL_SEPARATION_DIS;
        int NUMBER_OF_JOINTS;
        double pose_x_= 0.0;  // pose odometry
        double pose_y_= 0.0;  // pose odometry
        double psi_ = 0.0; // pose odometry
        double prev_front_wheel_angle = 0.0;
        double prev_steering_angle = 0.0;
        double steering_angle = 0.0;
        double des_theta = 0.0; 
        std::string joint_names_array_[3];
        int joint_orientation_array_[3];
        std::list<std::string> joint_names_;
        std::list<std::string> joint_map_key_;
        std::string NAME_SPACE;
        // std::vector<int> joint_orientation_;
        std::string joint_name;

        bool scooter_state_;      // Current loop state
        bool prev_scooter_state_; // Previous loop state
        ros::Time current_time_odometry_, last_time_odometry_, current_time_cmd_vel_, last_time_cmd_vel_, current_time_js_, last_time_js_;
        // tf::TransformBroadcaster odom_broadcaster;

        // Topics we are subscribing
        ros::Subscriber joint_state_sub_;
        void jointStateCallback( const sensor_msgs::JointStateConstPtr& msg );
        ros::Subscriber cmd_vel_sub_;
        void cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg );
        ros::Subscriber imu_sub_;
        void imuCallback( const sensor_msgs::ImuConstPtr &imu_msg );

        // Topics we are publishing
        ros::Publisher odom_pub_;
        ros::Publisher twist_pub_;
        ros::Publisher handle_bar_jpc, rotational_plate_jec, front_wheel_jvc;

        // Services we call
        ros::ServiceClient imu_calibrate_;
        std_srvs::Empty calibrate_;

        // Node Handle
        ros::NodeHandle nh_;
};

#endif // CONTROL_H_