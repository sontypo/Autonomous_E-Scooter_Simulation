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

void Dynamics::updateDynamics(const escooter_msgs::Pose &body_frame, escooter_msgs::Joints *joints, double tau, double Vx)
{
    current_time_dynamics_ = ros::Time::now();
    double dt = ( current_time_dynamics_ - last_time_dynamics_ ).toSec();

    I_theta = 4/3 * WHOLE_BODY_WEIGHT * pow(COM_POS_Z,2) * UNCERTAINTIES;
    I_rp_x = pow(0.036,2) * R_PLATE_WEIGHT * UNCERTAINTIES;
    I_rp_y = 0.0;
    I_rp_z = pow(0.076,2) * R_PLATE_WEIGHT * UNCERTAINTIES;

    theta_ = body_frame.orientation.roll;
    Trig T = getSinCos(theta_);          
    double cos_theta = T.cosine;
    double sin_theta = T.sine;

    theta_2dot_ = ( tau + ( WHOLE_BODY_WEIGHT * COM_POS_Z * B_WHEEL_TO_COM * psi_2dot_ * cos_theta ) 
                       + ( WHOLE_BODY_WEIGHT * COM_POS_Z * psi_dot_ * cos_theta * ( Vx - COM_POS_Z * psi_dot_ * sin_theta ) ) 
                       + WHOLE_BODY_WEIGHT * g * COM_POS_Z * sin_theta ) / ( I_theta + I_rp_x + I_rp_z );

    theta_dot_ += theta_2dot_ * dt;
    last_time_dynamics_ = current_time_dynamics_;
    
    joints->rotational_plate = theta_dot_;
}

void Dynamics::constraints(double Vx, double Vx_dot, double delta, double delta_dot)
{
    psi_dot_ = tan(delta) * Vx / WHEEL_SEPARATION_DIS;
    psi_2dot_ = delta_dot * (1 + tan(delta) * tan(delta)) / WHEEL_SEPARATION_DIS + Vx_dot * tan(delta) / WHEEL_SEPARATION_DIS;
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