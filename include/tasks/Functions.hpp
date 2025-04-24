#pragma once

#include <iostream>
#include <unistd.h>
#include <string.h>
#include <string>
#include <chrono>
#include <linux/can/raw.h>
#include <fstream>
#include <memory>
#include <iomanip>
#include <sstream>

#include "Motor.hpp"

#define SEND_SIGN 100
#define INIT_SIGN 99.9

using namespace std;

class Functions
{
public:
    Functions(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    ~Functions();

    struct Event{
    double time;
    std::vector<int> notes;
    };

    // restart CAN Port
    int getComNumberByHostname();
    void restCanPort();

    // save csv file
    std::chrono::high_resolution_clock::time_point start;  
    const std::string basePath = "../../DrumRobot_data/";  // 기본 경로
    std::string file_name = "data";
    void openCSVFile();
    void appendToCSV_DATA(const std::string& filename, float A_DATA, float B_DATA, float C_DATA);
    void appendToCSV_DATA_absTime(const std::string& filename, float A_DATA, float B_DATA, float C_DATA);
    void appendToCSV_CAN(const std::string& filename, can_frame& c_frame);
    void appendToCSV_time(const std::string& filename);
    void appendToCSV_State(const std::string& filename, string state, string sub_state);

    //midi to chord parsing

    //csv parsing
    std::vector<std::string> splitByWhitespace(const std::string& line);
    bool readMidiFile(const std::string& filename, std::vector<unsigned char>& buffer);
    
    size_t readTime(const std::vector<unsigned char>& data, size_t& pos);
    void handleMetaEvent(const std::vector<unsigned char>& data, size_t& pos, int& initial_setting_flag);
    void handleChannel10(const std::vector<unsigned char>& data, size_t& pos, unsigned char eventType);
    void handleNoteOn(const std::vector<unsigned char>& data, size_t& pos, double& note_on_time, int tpqn, const std::string& midiFilePath);
    void analyzeMidiEvent(const std::vector<unsigned char>& data, size_t& pos, unsigned char& runningStatus, int& initial_setting_flag, double& note_on_time, int& tpqn, const std::string& midiFilePath);
    
    void convertMcToC(const std::string& inputFilename, const std::string& outputFilename);
    void assignHandsToEvents(const std::string& inputFilename, const std::string& outputFilename);
    void convertToMeasureFile(const std::string& inputFilename, const std::string& outputFilename);

    
    void save_to_csv(const std::string& outputCsvPath, double& note_on_time, int drumNote);
   


private:
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;   // data 기록하기 위해 필요
};