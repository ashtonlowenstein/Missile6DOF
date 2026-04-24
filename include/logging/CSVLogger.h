//
// Created by Ashton Lowenstein on 4/20/26.
//

#ifndef MISSILE6DOF_CSVLOGGER_H
#define MISSILE6DOF_CSVLOGGER_H
#include <string>
#include <fstream>

#include "LogRecord.h"


class CSVLogger {
public:
    explicit CSVLogger(const std::string& filename);
    void log(const LogRecord& r);
private:
    void writeHeader();
    std::ofstream file_;
};


#endif //MISSILE6DOF_CSVLOGGER_H