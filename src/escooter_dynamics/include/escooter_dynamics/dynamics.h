#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <cmath>
#include <limits>
#include <algorithm>
#include <ros/ros.h>
#include <escooter_msgs/Joints.h>
#include <escooter_msgs/Pose.h>
#include <escooter_msgs/RPY.h>

//=============================================================================
// Define structs and classes for Dynamics system
//=============================================================================

struct Trig
{
    double sine;
    double cosine;
};

class Dynamics
{
    public:
        Dynamics(void);
        void updateDynamics(double theta, escooter_msgs::Joints *joints, double tau, double Vx);
        void constraints(double Vx, double Vx_dot, double delta, double delta_dot);
        // theta_dot_measured: body roll rate [rad/s] straight from the IMU gyro.
        //   Pass NaN to fall back to differentiating the roll angle.
        // plate_velocity: reaction plate speed [rad/s], for momentum management.
        void balancingController(const escooter_msgs::Pose &body_frame, double Vx,
                                 double theta_dot_measured = std::numeric_limits<double>::quiet_NaN(),
                                 double plate_velocity = 0.0);
        double computeCOMVelocity(double Vx, double theta, double theta_dot, double psi, double psi_dot);
        double psi_dot_, psi_2dot_;
        double V_x, V_y, V_z;
        double theta_ = 0.0;
        double tau_ = 0.0;

    private:
        Trig getSinCos(double rad_angle);
        ros::Time current_time_dynamics_, last_time_dynamics_, current_time_blc_, last_time_blc_;
        double COM_POS_X, COM_POS_Y, COM_POS_Z; // intitial position for COM
        double BODY_WEIGHT, R_PLATE_WEIGHT, F_WHEEL_WEIGHT, HANDLE_BAR_WEIGHT, WHOLE_BODY_WEIGHT, WHOLE_BODY_WEIGHT_EXC;
        double WHEEL_RADIUS, WHEEL_SEPARATION_DIS, B_WHEEL_TO_COM;
        double PD_PDplus, KP, KD;
        double MAX_BALANCING_TORQUE;    // saturation of tau_ [N.m]
        double MAX_MODEL_ROLL_RATE;     // bound on the open-loop model state [rad/s]
        double MOMENTUM_UNLOAD_GAIN;    // rad of lean per rad/s of plate speed
        double MAX_LEAN_SETPOINT;       // hard bound on the biased setpoint [rad]
        double theta_ref_;              // biased upright setpoint, for logging
        double UNCERTAINTIES;
        double C_, G_, CpG_;
        double theta_dot_, pre_theta_;
        double theta_2dot_, I_theta, I_rp_x, I_rp_y, I_rp_z;
};

#endif // DYNAMICS_H_