//
// Created by Ashton Lowenstein on 5/21/26.
//

#include "logging/GainTuningCSVLogger.h"

GainTuningCSVLogger::GainTuningCSVLogger(const std::string &filename) {
    file_.open(filename);
    if (!file_) {
        throw std::runtime_error("Failed to open log file: " + filename);
    }
    writeHeader();
}

void GainTuningCSVLogger::writeHeader() {
    file_
        << "x,y,z,"
        << "vx,vy,vz,"
        << "tilt,"
        << "gimbal_pitch_cmd,gimbal_yaw_cmd\n";
}

void GainTuningCSVLogger::log(const LogRecord &r) {
    file_
        << r.pos_inertial.x << "," << r.pos_inertial.y << "," << r.pos_inertial.z << ","
        << r.vel_inertial.x << "," << r.vel_inertial.y << "," << r.vel_inertial.z << ","
        << r.tilt << ","
        << r.gimbal_pitch_cmd << "," << r.gimbal_yaw_cmd
        << "\n";
}
