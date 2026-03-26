#include "../include/managers/SilCommandPipeWriter.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <sys/uio.h>
#include <unistd.h>

SilCommandPipeWriter::SilCommandPipeWriter(const std::string &pipePath)
    : pipePath(pipePath)
{
}

SilCommandPipeWriter::~SilCommandPipeWriter()
{
    closePipe();
}

void SilCommandPipeWriter::setEnabled(bool enabled)
{
    this->enabled = enabled;
    if (!this->enabled)
    {
        closePipe();
    }
}

bool SilCommandPipeWriter::openPipe()
{
    if (!enabled)
    {
        return false;
    }

    if (pipeFd >= 0)
    {
        return true;
    }

    pipeFd = open(pipePath.c_str(), O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    if (pipeFd >= 0)
    {
        openFailureLogged = false;
        return true;
    }

    if (!openFailureLogged)
    {
        std::cerr << "[SIL] Pipe open failed: " << pipePath
                  << " (" << strerror(errno) << ")" << std::endl;
        openFailureLogged = true;
    }

    return false;
}

void SilCommandPipeWriter::closePipe()
{
    if (pipeFd >= 0)
    {
        close(pipeFd);
        pipeFd = -1;
    }
}

void SilCommandPipeWriter::writeTMotor(const std::string &motorName, TMotor &motor, const TMotorData &data)
{
    if (!enabled)
    {
        return;
    }

    if (!openPipe())
    {
        return;
    }

    const float jointTargetDeg = motorTypeConversion(motor, data);
    writeLine(buildTMotorLine(motorName, data, jointTargetDeg));
}

void SilCommandPipeWriter::writeMaxon(const std::string &motorName, MaxonMotor &motor, const MaxonData &data)
{
    if (!enabled)
    {
        return;
    }

    if (!openPipe())
    {
        return;
    }

    const float jointTargetDeg = motorTypeConversion(motor, data);
    writeLine(buildMaxonLine(motorName, data, jointTargetDeg));
}

void SilCommandPipeWriter::writeDxl(const std::string &motorName, float position)
{
    if (!enabled)
    {
        return;
    }

    if (!openPipe())
    {
        return;
    }

    writeLine(buildDxlLine(motorName, motorTypeConversion(position)));
}

void SilCommandPipeWriter::writeLine(const std::string &line)
{
    if (pipeFd < 0)
    {
        return;
    }

    static char newline = '\n';

    // 기대하는 총 바이트 수 = line의 바이트 수 + 개행 문자 1바이트
    const struct iovec iov[2] = {
        {const_cast<char *>(line.data()), line.size()},
        {const_cast<char *>(&newline), 1},
    };
    const ssize_t expectedBytes = static_cast<ssize_t>(line.size()) + 1;

    while (true)
    {
        // writev를 사용하여 line과 개행 문자를 한 번에 씁니다.
        const ssize_t written = writev(pipeFd, iov, 2);
        
        if (written == expectedBytes)
        {
            return;
        }

        if (written < 0 && errno == EINTR)
        {
            continue;
        }

        if (written < 0 && errno == EPIPE)
        {
            closePipe();
        }

        return;
    }
}

float SilCommandPipeWriter::motorTypeConversion(TMotor &motor, const TMotorData &data) const
{
    const float jointTargetRad = motor.motorPositionToJointAngle(data.position);
    return jointTargetRad * 180.0f / static_cast<float>(M_PI);
}

float SilCommandPipeWriter::motorTypeConversion(MaxonMotor &motor, const MaxonData &data) const
{
    const float jointTargetRad = motor.motorPositionToJointAngle(data.position);
    return jointTargetRad * 180.0f / static_cast<float>(M_PI);
}

float SilCommandPipeWriter::motorTypeConversion(float dxlPositionRad) const
{
    return dxlPositionRad * 180.0f / static_cast<float>(M_PI);
}

std::string SilCommandPipeWriter::buildTMotorLine(const std::string &motorName, const TMotorData &data, float jointTargetDeg) const
{
    std::ostringstream stream;
    stream << "{\"kind\":\"tmotor\","
           << "\"motor\":\"" << motorName << "\","
           << "\"position\":" << jointTargetDeg << ","
           << "\"velocityERPM\":" << data.velocityERPM << ","
           << "\"mode\":" << data.mode << ","
           << "\"useBrake\":" << data.useBrake
           << "}";

    return stream.str();
}

std::string SilCommandPipeWriter::buildMaxonLine(const std::string &motorName, const MaxonData &data, float jointTargetDeg) const
{
    std::ostringstream stream;
    stream << "{\"kind\":\"maxon\","
           << "\"motor\":\"" << motorName << "\","
           << "\"position\":" << jointTargetDeg << ","
           << "\"mode\":" << data.mode << ","
           << "\"kp\":" << data.kp << ","
           << "\"kd\":" << data.kd
            << "}";
    return stream.str();
}

std::string SilCommandPipeWriter::buildDxlLine(const std::string &motorName, float position) const
{
    std::ostringstream stream;
    stream << "{\"kind\":\"dxl\","
           << "\"motor\":\"" << motorName << "\","
           << "\"position\":" << position
           << "}";
    return stream.str();
}
