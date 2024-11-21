#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <cmath>
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
        void balancingController(const escooter_msgs::Pose &body_frame, double Vx);
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
        double UNCERTAINTIES;
        double C_, G_, CpG_;
        double theta_dot_, pre_theta_;
        double theta_2dot_, I_theta, I_rp_x, I_rp_y, I_rp_z;
};

#endif // DYNAMICS_H_