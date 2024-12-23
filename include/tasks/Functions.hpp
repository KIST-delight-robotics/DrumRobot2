#pragma once

#include <iostream>
#include <unistd.h>
#include <string.h>
#include <string>
#include <chrono>
#include <linux/can/raw.h>
#include <fstream>
#include <memory>

#include "Motor.hpp"

#define SEND_SIGN 100
#define INIT_SIGN 99.9

using namespace std;

class Functions
{
public:
    Functions(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    ~Functions();

    // restart CAN Port
    int get_com_number_by_hostname();
    void restCanPort(int com_number);

    /*save csv file*/
    std::chrono::high_resolution_clock::time_point start;  
    const std::string basePath = "../../DrumRobot_data/";  // 기본 경로
    std::string file_name = "data";
    void openCSVFile();
    void appendToCSV_DATA(const std::string& filename, float A_DATA, float B_DATA, float C_DATA);
    void appendToCSV_CAN(const std::string& filename, can_frame& c_frame);
    void appendToCSV_time(const std::string& filename);
    void appendToCSV_State(const std::string& filename, string state, string sub_state);

private:
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
};