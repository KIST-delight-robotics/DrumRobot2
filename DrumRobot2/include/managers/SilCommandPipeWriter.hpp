#ifndef SIL_COMMAND_PIPE_WRITER_HPP
#define SIL_COMMAND_PIPE_WRITER_HPP

#include <string>
#include "Motor.hpp"

class SilCommandPipeWriter
{
public:
    explicit SilCommandPipeWriter(const std::string &pipePath = "/tmp/drum_command.pipe");
    ~SilCommandPipeWriter();

    void setEnabled(bool enabled);
    bool openPipe();
    void closePipe();

    void writeTMotor(const std::string &motorName, TMotor &motor, const TMotorData &data);
    void writeMaxon(const std::string &motorName, MaxonMotor &motor, const MaxonData &data);
    void writeDxl(const std::string &motorName, float position);
    void writeTick();

private:
    std::string pipePath;
    int pipeFd = -1;
    bool enabled = false;
    bool openFailureLogged = false;

    void writeLine(const std::string &line);

    float motorTypeConversion(TMotor &motor, const TMotorData &data) const;
    float motorTypeConversion(MaxonMotor &motor, const MaxonData &data) const;
    float motorTypeConversion(float dxlPositionRad) const;

    std::string buildTMotorLine(const std::string &motorName, const TMotorData &data, float jointTargetDeg) const;
    std::string buildMaxonLine(const std::string &motorName, const MaxonData &data, float jointTargetDeg) const;
    std::string buildDxlLine(const std::string &motorName, float position) const;
};

#endif
