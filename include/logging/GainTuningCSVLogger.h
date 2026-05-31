//
// Created by Ashton Lowenstein on 5/21/26.
//

#ifndef MISSILE6DOF_GAINTUNINGCSVLOGGER_H
#define MISSILE6DOF_GAINTUNINGCSVLOGGER_H

#include <string>
#include <fstream>

#include "LogRecord.h"

class GainTuningCSVLogger {
public:
    explicit GainTuningCSVLogger(const std::string& filename);
    void log(const LogRecord &r);
private:
    void writeHeader();
    std::ofstream file_;
};

#endif //MISSILE6DOF_GAINTUNINGCSVLOGGER_H