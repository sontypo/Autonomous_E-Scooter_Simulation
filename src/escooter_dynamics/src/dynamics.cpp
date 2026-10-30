#include <escooter_dynamics/dynamics.h>

const double PI = atan(1.0)*4.0;
const double g = 9.81;

//==============================================================================
//  Constructor: Initialize dynamics variables
//==============================================================================

Dynamics::Dynamics(void)
{
    ros::param::get( "COM_POS_X", COM_POS_X );
    ros::param::get( "COM_POS_Y", COM_POS_Y );
    ros::param::get( "COM_POS_Z", COM_POS_Z );
    ros::param::get( "BODY_WEIGHT", BODY_WEIGHT );
    ros::param::get( "R_PLATE_WEIGHT", R_PLATE_WEIGHT );
    ros::param::get( "F_WHEEL_WEIGHT", F_WHEEL_WEIGHT );
    ros::param::get( "HANDLE_BAR_WEIGHT", HANDLE_BAR_WEIGHT );
    ros::param::get( "WHOLE_BODY_WEIGHT", WHOLE_BODY_WEIGHT );
    ros::param::get( "WHOLE_BODY_WEIGHT_EXC", WHOLE_BODY_WEIGHT_EXC );
    ros::param::get( "WHEEL_RADIUS", WHEEL_RADIUS );
    ros::param::get( "WHEEL_SEPARATION_DIS", WHEEL_SEPARATION_DIS );
    ros::param::get( "UNCERTAINTIES", UNCERTAINTIES );
    ros::param::get( "B_WHEEL_TO_COM", B_WHEEL_TO_COM );
    ros::param::get( "PD_PDplus", PD_PDplus );
    ros::param::get( "KP", KP );
    ros::param::get( "KD", KD );

    // Defaults so these stay defined when the YAML predates them.
    MAX_BALANCING_TORQUE = 150.0;
    MAX_MODEL_ROLL_RATE  = 50.0;
    MOMENTUM_UNLOAD_GAIN = 0.0005;
    MAX_LEAN_SETPOINT    = 0.035;
    theta_ref_           = 0.0;
    ros::param::get( "MAX_BALANCING_TORQUE", MAX_BALANCING_TORQUE );
    ros::param::get( "MAX_MODEL_ROLL_RATE", MAX_MODEL_ROLL_RATE );
    ros::param::get( "MOMENTUM_UNLOAD_GAIN", MOMENTUM_UNLOAD_GAIN );
    ros::param::get( "MAX_LEAN_SETPOINT", MAX_LEAN_SETPOINT );
}

//=============================================================================
// getSinCos:  Get the sinus and cosinus from the angle
//=============================================================================

Trig Dynamics::getSinCos( double angle_rad )
{
    Trig body_trig;

    body_trig.sine = sin( angle_rad );
    body_trig.cosine = cos( angle_rad );

    return body_trig;
}

//=============================================================================
// Dynamics Model
//=============================================================================

void Dynamics::updateDynamics(double theta, escooter_msgs::Joints *joints, double tau, double Vx)
{
    current_time_dynamics_ = ros::Time::now();
    double dt = ( current_time_dynamics_ - last_time_dynamics_ ).toSec();

    // FIX: 4/3 is integer division in C++ and evaluates to 1, which made this
    // term 25% too small.
    I_theta = 4.0/3.0 * WHOLE_BODY_WEIGHT * pow(COM_POS_Z,2) * UNCERTAINTIES;
    I_rp_x = pow(0.036,2) * R_PLATE_WEIGHT * UNCERTAINTIES;
    I_rp_y = 0.0;
    I_rp_z = pow(0.076,2) * R_PLATE_WEIGHT * UNCERTAINTIES;

    Trig T = getSinCos(theta);          
    double cos_theta = T.cosine;
    double sin_theta = T.sine;

    theta_2dot_ = ( tau + ( WHOLE_BODY_WEIGHT * COM_POS_Z * B_WHEEL_TO_COM * psi_2dot_ * cos_theta ) 
                       + ( WHOLE_BODY_WEIGHT * COM_POS_Z * psi_dot_ * cos_theta * ( Vx - COM_POS_Z * psi_dot_ * sin_theta ) ) 
                       + WHOLE_BODY_WEIGHT * g * COM_POS_Z * sin_theta ) / ( I_theta + I_rp_x + I_rp_z );

    theta_dot_ += theta_2dot_ * dt;
    // The forward model runs open loop, so bound its state. It is a prediction
    // kept for analysis, not a command.
    theta_dot_ = std::max(-MAX_MODEL_ROLL_RATE, std::min(MAX_MODEL_ROLL_RATE, theta_dot_));
    last_time_dynamics_ = current_time_dynamics_;

    // FIX: command the balancing torque that balancingController() computed.
    // This used to publish theta_dot_, the running integral of the modelled roll
    // acceleration, in rad/s, straight into an effort controller expecting N.m.
    // Nothing reset that integral, so the command ramped without bound: a 60 s
    // run reached -348743 N.m against a 500 N.m joint limit.
    joints->rotational_plate = tau;
}

void Dynamics::constraints(double Vx, double Vx_dot, double delta, double delta_dot)
{
    psi_dot_ = tan(delta) * Vx / WHEEL_SEPARATION_DIS;
    psi_2dot_ = delta_dot * (1 + tan(delta) * tan(delta)) / WHEEL_SEPARATION_DIS + Vx_dot * tan(delta) / WHEEL_SEPARATION_DIS;
}

void Dynamics::balancingController(const escooter_msgs::Pose &body_frame, double Vx,
                                   double theta_dot_measured, double plate_velocity)
{
    current_time_blc_ = ros::Time::now();
    double dt = ( current_time_blc_ - last_time_blc_ ).toSec();

    theta_ = body_frame.orientation.roll;
    double delta_theta = theta_ - pre_theta_;
    double theta_dot_imu = delta_theta / dt;
    last_time_blc_ = current_time_blc_;
    pre_theta_ = theta_;

    // FIX: prefer the gyro reading. The finite difference above samples the roll
    // angle at MASTER_LOOP_RATE (500 Hz) while the IMU publishes at 100 Hz, so it
    // is zero four cycles out of five and five times too large on the fifth - a
    // spike train that the D term amplifies.
    if ( std::isfinite(theta_dot_measured) ) theta_dot_imu = theta_dot_measured;

    Trig T = getSinCos(theta_);          
    double cos_theta = T.cosine;
    double sin_theta = T.sine;

    C_ = ( WHOLE_BODY_WEIGHT * COM_POS_Z * B_WHEEL_TO_COM * psi_2dot_ ) 
                       + ( WHOLE_BODY_WEIGHT * COM_POS_Z * psi_dot_ * ( Vx - COM_POS_Z * psi_dot_ * sin_theta ) );
    G_ = WHOLE_BODY_WEIGHT * g * COM_POS_Z;
    CpG_ = C_ * cos_theta + G_ * sin_theta;

    // FIX: sign of the stabilising feedback.
    //
    // Measured in Gazebo: a positive effort on rotational_plate_joint reacts onto
    // the body as a *negative* roll torque (tau = +40 N.m turned a +1.70 rad/s
    // free-fall roll rate into -0.32 rad/s). Cancelling a positive roll therefore
    // needs a positive plate torque, so the feedback carries the same sign as the
    // gravity feed-forward.
    //
    // This used to be `0.0 - theta_`, which subtracted it. With G_ = M*g*h =
    // 72.7 N.m/rad against KP = 2.0, the loop had an effective stiffness of
    // +70.7 N.m/rad: it drove the scooter over instead of holding it up.
    //
    // Momentum management: the chassis centre of mass sits a few millimetres off
    // the centreline, so gravity applies a constant roll torque that the plate
    // would otherwise absorb until it saturates (measured: 7.6 rad/s per second,
    // falling over after ~26 s). Biasing the setpoint by the plate speed lets the
    // body hold a fraction of a degree of lean that cancels it. The sign follows
    // from the equilibrium of the two bodies: a lean theta_ref accelerates the
    // plate at G*sin(theta_ref)/I_w, so bleeding a positive plate speed needs a
    // negative lean. Set MOMENTUM_UNLOAD_GAIN to 0 to hold exactly upright.
    theta_ref_ = -MOMENTUM_UNLOAD_GAIN * plate_velocity;
    theta_ref_ = std::max(-MAX_LEAN_SETPOINT, std::min(MAX_LEAN_SETPOINT, theta_ref_));

    double error_p = theta_ - theta_ref_;
    double error_d = theta_dot_imu;

    tau_ = ( PD_PDplus * CpG_ ) + ( KP * error_p ) + ( KD * error_d );
    tau_ = std::max(-MAX_BALANCING_TORQUE, std::min(MAX_BALANCING_TORQUE, tau_));
}

double Dynamics::computeCOMVelocity(double Vx, double theta, double theta_dot, double psi, double psi_dot)
{
    Trig T = getSinCos(theta);
    double cos_theta = T.cosine;
    double sin_theta = T.sine; 
    Trig P = getSinCos(psi);
    double cos_psi = P.cosine;
    double sin_psi = P.sine;

    V_x = (Vx * cos_theta) - (B_WHEEL_TO_COM * psi_dot * sin_psi) + (COM_POS_Z * theta_dot * cos_theta * sin_psi) + (COM_POS_Z * psi_dot * sin_theta * cos_psi);
    V_y = (Vx * sin_theta) + (B_WHEEL_TO_COM * psi_dot * cos_psi) - (COM_POS_Z * theta_dot * cos_theta * cos_psi) + (COM_POS_Z * psi_dot * sin_theta * sin_psi);
    V_z = - COM_POS_Z * theta_dot * sin_theta;

    return V_x, V_y, V_z;
}