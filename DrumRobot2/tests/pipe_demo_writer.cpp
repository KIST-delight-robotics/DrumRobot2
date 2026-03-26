#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>

struct DemoCommand
{
    std::string motorName;
    float position;
    int mode;
};

namespace
{
const char *kPipePath = "/tmp/drum_pipe_demo";

bool createPipeIfMissing()
{
    if (mkfifo(kPipePath, 0666) == 0)
    {
        return true;
    }

    return errno == EEXIST;
}

std::string buildLine(const DemoCommand &command)
{
    std::ostringstream stream;
    stream << "{\"kind\":\"demo\","
           << "\"motor\":\"" << command.motorName << "\","
           << "\"position\":" << command.position << ","
           << "\"mode\":" << command.mode
           << "}";
    return stream.str();
}
} // namespace

int main()
{
    if (!createPipeIfMissing())
    {
        std::cerr << "mkfifo failed: " << std::strerror(errno) << '\n';
        return 1;
    }

    std::cout << "Waiting for Python reader on " << kPipePath << "...\n";
    const int pipeFd = open(kPipePath, O_WRONLY);
    if (pipeFd < 0)
    {
        std::cerr << "open failed: " << std::strerror(errno) << '\n';
        return 1;
    }

    const DemoCommand commands[] = {
        {"R_arm1", 30.0f, 1},
        {"R_wrist", 45.0f, 2},
    };

    for (const DemoCommand &command : commands)
    {
        const std::string line = buildLine(command) + "\n";
        const ssize_t written = write(pipeFd, line.c_str(), line.size());
        if (written < 0)
        {
            std::cerr << "write failed: " << std::strerror(errno) << '\n';
            close(pipeFd);
            return 1;
        }

        std::cout << "Sent: " << line;
    }

    close(pipeFd);
    std::cout << "Done.\n";
    return 0;
}
