//
// Created by Ashton Lowenstein on 4/20/26.
//

#include "../../include/logging/CSVLogger.h"

CSVLogger::CSVLogger(const std::string &filename) {
    file_.open(filename);
    if (!file_) {
        throw std::runtime_error("Failed to open log file: " + filename);
    }
    writeHeader();
}

void CSVLogger::writeHeader() {
    file_
            << "t,"
            << "x,y,z,"
            << "x_hat,y_hat,z_hat,"
            << "vx,vy,vz,"
            << "vx_hat,vy_hat,vz_hat,"
            << "qw,qx,qy,qz,"
            << "qw_hat,qx_hat,qy_hat,qz_hat,"
            << "omega_x,omega_y,omega_z,"
            << "omegax_hat,omegay_hat,omegaz_hat,"
            << "omega_dot_x,omega_dot_y,omega_dot_z,"
            << "mass,"
            << "throttle_cmd,throttle,"
            << "thrust,"
            << "gimbal_pitch_cmd,gimbal_pitch,"
            << "gimbal_yaw_cmd,gimbal_yaw,"
            << "q_cmd_w,q_cmd_x,q_cmd_y,q_cmd_z,q_error_norm,"
            << "a_body_x,a_body_y,a_body_z,"
            << "a_body_bias_x,a_body_bias_y,a_body_bias_z,"
            << "Fx,Fy,Fz,"
            << "Mx,My,Mz,"
            << "mdot,"
            << "air_speed,alpha,beta,mach,qbar,"
            << "Ftot_x,Ftot_y,Ftot_z,"
            << "Mtot_x,Mtot_y,Mtot_z,"
            << "Ixx,Iyy,Izz\n";
}

void CSVLogger::log(const LogRecord &r) {
    file_
            << r.t << ","
            << r.pos_inertial.x << "," << r.pos_inertial.y << "," << r.pos_inertial.z << ","
            << r.pos_inertial_estimate.x << "," << r.pos_inertial_estimate.y << "," << r.pos_inertial_estimate.z << ","
            << r.vel_inertial.x << "," << r.vel_inertial.y << "," << r.vel_inertial.z << ","
            << r.vel_inertial_estimate.x << "," << r.vel_inertial_estimate.y << "," << r.vel_inertial_estimate.z << ","
            << r.q_bi.w << "," << r.q_bi.x << "," << r.q_bi.y << "," << r.q_bi.z << ","
            << r.q_bi_estimate.w << "," << r.q_bi_estimate.x << "," << r.q_bi_estimate.y << "," << r.q_bi_estimate.z << ","
            << r.omega_body.x << "," << r.omega_body.y << "," << r.omega_body.z << ","
            << r.omega_body_estimate.x << "," << r.omega_body_estimate.y << "," << r.omega_body_estimate.z << ","
            << r.omega_body_dot.x << "," << r.omega_body_dot.y << "," << r.omega_body_dot.z << ","
            << r.mass << ","

            << r.throttle_cmd << "," << r.throttle_actual << ","
            << r.thrust << ","
            << r.gimbal_pitch_cmd << "," << r.gimbal_pitch_actual << ","
            << r.gimbal_yaw_cmd << "," << r.gimbal_yaw_actual << ","

            << r.q_cmd.w << "," << r.q_cmd.x << "," << r.q_cmd.y << "," << r.q_cmd.z << "," << r.q_error_norm << ","

            << r.accel_body.x << "," << r.accel_body.y << "," << r.accel_body.z << ","
            << r.accel_body_bias.x << "," << r.accel_body_bias.y << "," << r.accel_body_bias.z << ","

            << r.thrust_force_body.x << "," << r.thrust_force_body.y << "," << r.thrust_force_body.z << ","
            << r.thrust_moment_body.x << "," << r.thrust_moment_body.y << "," << r.thrust_moment_body.z << ","
            << r.mdot << ","

            << r.air_speed << "," << r.alpha << "," << r.beta << "," << r.mach << "," << r.qbar << ","

            << r.total_force_body.x << "," << r.total_force_body.y << "," << r.total_force_body.z << ","
            << r.total_moment_body.x << "," << r.total_moment_body.y << "," << r.total_moment_body.z << ","

            << r.Ixx << "," << r.Iyy << "," << r.Izz
            << "\n";
}
