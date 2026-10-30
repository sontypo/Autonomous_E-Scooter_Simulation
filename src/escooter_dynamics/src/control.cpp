#include <cmath>
#include <algorithm>
#include <limits>
#include <string>
#include <escooter_dynamics/control.h>

const double PI = atan(1.0)*4.0;

//==============================================================================
// Constructor
//==============================================================================

Control::Control(void)
{
    ros::param::get( "NAME_SPACE", NAME_SPACE);
    ros::param::get( "BODY_MAX_ROLL", BODY_MAX_ROLL );
    ros::param::get( "BODY_MAX_PITCH", BODY_MAX_PITCH );
    ros::param::get( "BODY_MAX_YAW", BODY_MAX_YAW );
    ros::param::get( "MAX_BODY_ROLL_COMP", MAX_BODY_ROLL_COMP );
    ros::param::get( "MAX_BODY_PITCH_COMP", MAX_BODY_PITCH_COMP );
    ros::param::get( "COMPENSATE_INCREMENT", COMPENSATE_INCREMENT );
    ros::param::get( "COMPENSATE_TO_WITHIN", COMPENSATE_TO_WITHIN );
    ros::param::get( "MASTER_LOOP_RATE", MASTER_LOOP_RATE );
    ros::param::get( "VELOCITY_DIVISION", VELOCITY_DIVISION );
    ros::param::get( "NUMBER_OF_JOINTS", NUMBER_OF_JOINTS );
    ros::param::get( "WHEEL_RADIUS", WHEEL_RADIUS );
    ros::param::get( "WHEEL_SEPARATION_DIS", WHEEL_SEPARATION_DIS );
    ros::param::get( "MAX_STEERING_ANGLE", MAX_STEERING_ANGLE );

    // Defaults, so these stay defined when the YAML predates them.
    STEERING_MODE            = "yaw_rate";
    STEERING_MIN_SPEED       = 0.5;
    MAX_STEERING_RATE        = 6.0;
    MAX_LINEAR_ACCEL         = 1.5;
    WHEEL_RATE_FROM_SPEED    = true;
    STEER_BALANCE_KP         = 0.0;
    STEER_BALANCE_KD         = 0.0;
    STEER_BALANCE_MAX        = 0.35;
    STEER_BALANCE_MIN_SPEED  = 1.0;
    STEER_BALANCE_FULL_SPEED = 2.0;
    WHOLE_BODY_WEIGHT        = 18.90654;
    COM_POS_Z                = 0.392;
    ros::param::get( "STEERING_MODE", STEERING_MODE );
    ros::param::get( "STEERING_MIN_SPEED", STEERING_MIN_SPEED );
    ros::param::get( "MAX_STEERING_RATE", MAX_STEERING_RATE );
    ros::param::get( "MAX_LINEAR_ACCEL", MAX_LINEAR_ACCEL );
    ros::param::get( "WHEEL_RATE_FROM_SPEED", WHEEL_RATE_FROM_SPEED );
    ros::param::get( "STEER_BALANCE_KP", STEER_BALANCE_KP );
    ros::param::get( "STEER_BALANCE_KD", STEER_BALANCE_KD );
    ros::param::get( "STEER_BALANCE_MAX", STEER_BALANCE_MAX );
    ros::param::get( "STEER_BALANCE_MIN_SPEED", STEER_BALANCE_MIN_SPEED );
    ros::param::get( "STEER_BALANCE_FULL_SPEED", STEER_BALANCE_FULL_SPEED );
    ros::param::get( "WHOLE_BODY_WEIGHT", WHOLE_BODY_WEIGHT );
    ros::param::get( "COM_POS_Z", COM_POS_Z );

    // Find out how many servos/joints we have
    for( int i = 0; i < NUMBER_OF_JOINTS; i++ )
    {
        ros::param::get( ("/JOINTS/" + std::to_string(i) + "/name"), joint_names_array_[i] );
        ros::param::get( ("/JOINTS/" + std::to_string(i) + "/sign"), joint_orientation_array_[i] );
    }

    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        ROS_INFO_STREAM("Joint_name["<<i<<"]: "<<joint_names_array_[i]<<"");
    }

    current_time_odometry_ = ros::Time::now();
    last_time_odometry_ = ros::Time::now();
    current_time_cmd_vel_ = ros::Time::now();
    last_time_cmd_vel_ = ros::Time::now();
    current_time_steer_ = ros::Time::now();
    last_time_steer_ = ros::Time::now();

    prev_scooter_state_ = false;
    scooter_state_ = false;
    imu_init_stored_ = false;
    imu_roll_lowpass_ = 0.0;
    imu_pitch_lowpass_ = 0.0;
    imu_yaw_lowpass_ = 0.0;
    imu_roll_init_ = 0.0;
    imu_pitch_init_ = 0.0;

    // Topics we are subscribing
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/e_scooter/joint_states", 1, &Control::jointStateCallback, this);
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "/e_scooter/cmd_vel", 1, &Control::cmd_velCallback, this );
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>( "/imu", 1, &Control::imuCallback, this );

    // Topics we are publishing
    handle_bar_jpc = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[0] + "_position_controller/command", 10);
    rotational_plate_jec = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[1] + "_effort_controller/command", 10);
    front_wheel_jvc = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[2] + "_velocity_controller/command", 10);
    // /e_scooter/rotational_plate_joint_effort_controller/command


    odom_pub_ = nh_.advertise<nav_msgs::Odometry>( "/e_scooter/odometry/calculated", 50 );
    twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>( "/twist", 50 );
}

//==============================================================================
// Getter and Setters
//==============================================================================

void Control::setScooterState(bool state)
{
    scooter_state_ = state;
}

bool Control::getScooterState(void)
{
    return scooter_state_;
}

void Control::setPrevScooterState(bool state)
{
    prev_scooter_state_ = state;
}

bool Control::getPrevScooterState(void)
{
    return prev_scooter_state_;
}

//==============================================================================
// Odometry Publisher
//==============================================================================

void Control::publishOdometry(double distance_traveled, double delta)
{
    // calculate time elapsed
    current_time_odometry_ = ros::Time::now();
    double dt = ( current_time_odometry_ - last_time_odometry_ ).toSec();

    double delta_psi = (distance_traveled / WHEEL_SEPARATION_DIS) * tan(delta);
    psi_ += delta_psi;

    double delta_x = distance_traveled * cos(psi_);
    double delta_y = distance_traveled * sin(psi_);
    pose_x_ += delta_x;
    pose_y_ += delta_y;

    double v_x = delta_x / dt;
    double v_y = delta_y / dt;
    double v_psi = delta_psi / dt;

    // Quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( psi_ );

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_odometry_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose_x_;
    odom_trans.transform.translation.y = pose_y_;
    odom_trans.transform.translation.z = body_frame_.orientation.yaw;
    odom_trans.transform.rotation = odom_quat;

    // Uncomment odom_broadcaster to send the transform. Only used if debugging calculated odometry.
    // odom_broadcaster.sendTransform( odom_trans );

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_odometry_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // set the position
    odom.pose.pose.position.x = pose_x_;
    odom.pose.pose.position.y = pose_y_;
    odom.pose.pose.position.z = body_frame_.orientation.yaw;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = 0.00001;  // x
    odom.pose.covariance[7] = 0.00001;  // y
    odom.pose.covariance[14] = 0.00001; // z
    odom.pose.covariance[21] = 0.001; // rot x
    odom.pose.covariance[28] = 0.001; // rot y
    odom.pose.covariance[35] = 0.001; // rot z

    // set the velocity
    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.angular.z = v_psi;
    odom.twist.covariance = odom.pose.covariance; // needed?

    odom_pub_.publish( odom );
    last_time_odometry_ = current_time_odometry_;
}

//==============================================================================
// Twist Publisher
//==============================================================================

void Control::publishTwist( const geometry_msgs::Twist &com_vel )
{
    geometry_msgs::TwistWithCovarianceStamped twistStamped;
    twistStamped.header.stamp = ros::Time::now();
    twistStamped.header.frame_id = "odom";

    twistStamped.twist.twist.linear.x = com_vel.linear.x;
    twistStamped.twist.twist.linear.y = com_vel.linear.y;
    twistStamped.twist.twist.angular.z = com_vel.angular.z;

    twistStamped.twist.covariance[0] = 0.00001;  // x
    twistStamped.twist.covariance[7] = 0.00001;  // y
    twistStamped.twist.covariance[14] = 0.00001; // z
    twistStamped.twist.covariance[21] = 1000000000000.0; // rot x
    twistStamped.twist.covariance[28] = 1000000000000.0; // rot y
    twistStamped.twist.covariance[35] = 0.001; // rot z

    twist_pub_.publish( twistStamped );
}

//==============================================================================
// Steering and speed, every control cycle
//==============================================================================

void Control::updateSteering()
{
    current_time_steer_ = ros::Time::now();
    double dt = ( current_time_steer_ - last_time_steer_ ).toSec();
    last_time_steer_ = current_time_steer_;

    // Ramp the speed command. front_wheel_joint is velocity controlled, so a
    // step makes its PID slam the drive torque, and the reaction tips a vehicle
    // balanced on two in-line wheels: stepping straight to 2 m/s took the wheel
    // from 0 to -11.5 rad/s in 200 ms and had the scooter at -67 degrees 200 ms
    // later.
    if (MAX_LINEAR_ACCEL > 0.0 && dt > 0.0) {
        const double max_step = MAX_LINEAR_ACCEL * dt;
        Vx_ += std::max(-max_step, std::min(max_step, rider_speed_ - Vx_));
    } else {
        Vx_ = rider_speed_;
    }
    // FIX: a real acceleration. This used to be Vx_/dt, a speed over a sample
    // interval - 40 m/s^2 at a steady 2 m/s with cmd_vel at 20 Hz, which
    // constraints() turned into a phantom 114 N.m of balancing feed-forward
    // whenever the scooter was driven and steered at once.
    Vx_dot_ = (dt > 0.0) ? ((Vx_ - prev_Vx_) / dt) : 0.0;
    prev_Vx_ = Vx_;

    // FIX: cmd_vel.linear.x is a ground speed in m/s but front_wheel_joint takes
    // a wheel rate in rad/s - the author's `TODO: need to convert to joint
    // command later`. Unconverted the scooter moved at v*WHEEL_RADIUS, 7.4 times
    // slower than asked, while the dynamics read the same number as a speed.
    joints_.front_wheel = (WHEEL_RATE_FROM_SPEED && WHEEL_RADIUS > 0.0) ?
                          (Vx_ / WHEEL_RADIUS) : Vx_;

    // Steer-to-balance. Shipped off (gains 0): implemented and sign-checked, but
    // measured not to rescue the scooter while driving. See docs on ros2-humble.
    balance_delta_ = 0.0;
    const double speed = std::abs(Vx_);
    double authority = 0.0;
    if (STEER_BALANCE_FULL_SPEED > STEER_BALANCE_MIN_SPEED) {
        authority = (speed - STEER_BALANCE_MIN_SPEED) /
                    (STEER_BALANCE_FULL_SPEED - STEER_BALANCE_MIN_SPEED);
        authority = std::max(0.0, std::min(1.0, authority));
    } else if (speed >= STEER_BALANCE_MIN_SPEED) {
        authority = 1.0;
    }
    if (authority > 0.0 && (STEER_BALANCE_KP != 0.0 || STEER_BALANCE_KD != 0.0)) {
        const double theta = body_frame_.orientation.roll;
        const double tau_wanted = -(STEER_BALANCE_KP * theta + STEER_BALANCE_KD * body_roll_rate_);
        const double lateral = WHOLE_BODY_WEIGHT * COM_POS_Z * speed * speed;
        if (lateral > 0.0) {
            balance_delta_ = authority * std::atan(WHEEL_SEPARATION_DIS * tau_wanted / lateral);
            balance_delta_ = std::max(-STEER_BALANCE_MAX, std::min(STEER_BALANCE_MAX, balance_delta_));
            if (Vx_ < 0.0) balance_delta_ = -balance_delta_;
        }
    }

    double target = rider_delta_ + balance_delta_;
    target = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, target));

    if (MAX_STEERING_RATE > 0.0 && dt > 0.0) {
        const double max_step = MAX_STEERING_RATE * dt;
        des_delta += std::max(-max_step, std::min(max_step, target - des_delta));
    } else {
        des_delta = target;
    }

    joints_.handle_bar = des_delta;
}

void Control::publishJointCommand(const escooter_msgs::Joints &joints)
{
    std_msgs::Float64 jpc_command;
    jpc_command.data = joint_orientation_array_[0] * joints.handle_bar;
    handle_bar_jpc.publish(jpc_command);

    std_msgs::Float64 jec_command;
    jec_command.data = joint_orientation_array_[1] * joints.rotational_plate;
    rotational_plate_jec.publish(jec_command);

    std_msgs::Float64 jvc_command;
    jvc_command.data = joint_orientation_array_[2] * joints.front_wheel;
    front_wheel_jvc.publish(jvc_command);

    // ROS_INFO_STREAM("handle_bar_jpc: "<<joints.handle_bar<<"");
    // ROS_INFO_STREAM("front_wheel_jvc: "<<joints.front_wheel<<"");
}

//==============================================================================
// Topics we subscribe to
//==============================================================================

//==============================================================================
// JointState callback
//==============================================================================
void Control::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    current_time_js_ = ros::Time::now();
    double dt = ( current_time_js_ - last_time_js_ ).toSec();

    std::string front_wheel_joint_name = "front_wheel_joint";
    std::string handle_bar_chasis_joint_name = "handle_bar_chasis_joint";
    std::string rotational_plate_joint_name = "rotational_plate_joint";
    double front_wheel_angle = prev_front_wheel_angle;
    double steering_velocity = std::numeric_limits<double>::quiet_NaN();

    // FIX: the loop used to `break` inside the first `if`, so whichever joint came
    // first in the message hid the others.
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
        if (msg->name[i] == front_wheel_joint_name) {
            front_wheel_angle = msg->position[i];
        } else if (msg->name[i] == handle_bar_chasis_joint_name) {
            steering_angle = msg->position[i];
            if (i < msg->velocity.size()) steering_velocity = msg->velocity[i];
        } else if (msg->name[i] == rotational_plate_joint_name && i < msg->velocity.size()) {
            plate_velocity_ = msg->velocity[i];
        }
    }

    // Calculate incremental distance traveled from wheel rotation
    double delta_angle = front_wheel_angle - prev_front_wheel_angle;
    double delta_distance = WHEEL_RADIUS * delta_angle;
    distance_traveled_ += delta_distance;    // Calculate the real feedback distance

    double delta_steering = steering_angle - prev_steering_angle;
    delta_ += delta_steering;
    // FIX: prefer the joint's reported velocity over differencing its position,
    // for the same reason the roll rate comes from the gyro - the quotient is
    // noisy and feeds the psi_2dot_ term of the balancing feed-forward.
    if ( std::isfinite(steering_velocity) ) delta_dot_ = steering_velocity;
    else delta_dot_ = (delta_ - prev_delta_) / dt;

    prev_front_wheel_angle = front_wheel_angle;
    prev_steering_angle = steering_angle;
    prev_delta_ = delta_;
    last_time_js_ = current_time_js_;
}


//==============================================================================
// cmd_vel callback
//==============================================================================

void Control::cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg )
{
    current_time_cmd_vel_ = ros::Time::now();
    double dt = ( current_time_cmd_vel_ - last_time_cmd_vel_ ).toSec();

    rider_speed_ = cmd_vel_msg->linear.x;
    const double omega = cmd_vel_msg->angular.z;

    // FIX: how angular.z becomes a steering angle.
    //
    // This used to be `des_delta += angular.z * dt`, with the author's own
    // `TODO: it should be delta` beside it. Teleop publishes a held value, so one
    // key press ramped the bars to the 45 degree stop over about 8 s and left
    // them there; releasing to angular.z = 0 froze them hard over instead of
    // centring, and reversing meant waiting out the same ramp backwards.
    //
    // The default now follows the ROS convention for cmd_vel on a bicycle:
    // angular.z is a yaw rate, inverted as delta = atan(L*omega/v). Centring is
    // immediate because omega = 0 gives delta = 0.
    double target_delta = 0.0;
    if (STEERING_MODE == "rate") {
        target_delta = rider_delta_ + omega * dt;       // legacy behaviour
    } else if (STEERING_MODE == "angle") {
        target_delta = omega;                           // angular.z is the angle
    } else {
        // The inversion is singular at standstill - a bicycle cannot change
        // heading while stopped - so the speed is floored, which keeps the bars
        // responsive while parked.
        double v_eff = rider_speed_;
        if (std::abs(v_eff) < STEERING_MIN_SPEED)
            v_eff = (v_eff < 0.0) ? -STEERING_MIN_SPEED : STEERING_MIN_SPEED;
        target_delta = std::atan(WHEEL_SEPARATION_DIS * omega / v_eff);
    }

    rider_delta_ = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, target_delta));

    last_time_cmd_vel_ = current_time_cmd_vel_;
}

//==============================================================================
// Override IMU and manipulate body orientation callback
//==============================================================================

//==============================================================================
// IMU callback to auto-level body if on non level ground
//==============================================================================

void Control::imuCallback( const sensor_msgs::ImuConstPtr &imu_msg )
{
    const geometry_msgs::Vector3 &lin_acc = imu_msg->linear_acceleration;

    if( imu_init_stored_ == false )
    {
        imu_roll_init_ = atan2( lin_acc.x, sqrt( lin_acc.y * lin_acc.y + lin_acc.z * lin_acc.z ) );
        imu_pitch_init_ = -atan2( lin_acc.y, lin_acc.z );
        imu_pitch_init_ = ( imu_pitch_init_ >= 0.0 ) ? ( PI - imu_pitch_init_ ) : ( -imu_pitch_init_ - PI );
        imu_init_stored_ = true;
    }

    const tf2::Quaternion quat( imu_msg->orientation.x,
                                imu_msg->orientation.y,
                                imu_msg->orientation.z,
                                imu_msg->orientation.w );

    double imu_roll, imu_pitch, imu_yaw;
    tf2::Matrix3x3(quat).getRPY(imu_roll, imu_pitch, imu_yaw);

    body_roll_rate_ = imu_msg->angular_velocity.x;

    body_frame_.orientation.roll = imu_roll;
    body_frame_.orientation.pitch = imu_pitch;
    body_frame_.orientation.yaw = imu_yaw;
}