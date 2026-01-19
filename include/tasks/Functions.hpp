#pragma once

#include <bits/stdc++.h>
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
#include <algorithm>

#include "Motor.hpp"

#define SEND_SIGN 100
#define INIT_SIGN 99

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
    std::chrono::steady_clock::time_point start;
    const std::string basePath = "../../DrumRobot_data/";  // 기본 경로
    std::string log_file_name = "data";
    void openCSVFile();
    std::ostringstream getAbsTime();
    void appendToCSV(const std::string& filename, bool useAbsTime);
    void appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA);
    void appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA);
    void appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA, float C_DATA);
    void appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA, float C_DATA, float D_DATA);
    void appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA, float C_DATA, float D_DATA, float E_DATA);
    void appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA, float C_DATA, float D_DATA, float E_DATA, float F_DATA);
    void appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA, float C_DATA, float D_DATA, float E_DATA, float F_DATA, float G_DATA);
    void appendToCSV(const std::string& filename, bool useAbsTime, can_frame& c_frame);
    void appendToCSV(const std::string& filename, bool useAbsTime, std::chrono::_V2::system_clock::time_point &__t);
    void appendToCSV(const std::string& filename, bool useAbsTime, std::vector<double> &__v);

    //midi to chord parsing

    enum Hand { LEFT, RIGHT, SAME };
    struct Coord {
        double x, y, z;
    };

    Coord drumXYZ[9] = {
        {0.0, 0.0, 0.0},
        {-0.061, 0.373, 0.512}, {0.284, 0.358, 0.532}, {0.244, 0.585, 0.628},
        {-0.031, 0.614, 0.692}, {-0.286, 0.504, 0.781}, {0.357, 0.625, 0.849},
        {0.496, 0.404, 0.747}, {-0.162, 0.676, 0.872}
    };

    double dist(const Coord& a, const Coord& b);
    //csv parsing
    std::vector<std::string> splitByWhitespace(const std::string& line);
    bool readMidiFile(const std::string& filename, std::vector<unsigned char>& buffer);

    void filterSmallDurations(const std::string& inputFilename, const std::string& outputFilename);
    void roundDurationsToStep(int bpm, const std::string& inputFilename, const std::string& outputFilename);
    size_t readTime(const std::vector<unsigned char>& data, size_t& pos);
    void handleMetaEvent(const std::vector<unsigned char>& data, size_t& pos, int &bpm);
    void handleChannel10(const std::vector<unsigned char>& data, size_t& pos, unsigned char eventType);
    void handleNoteOn(const std::vector<unsigned char>& data, size_t& pos, double& note_on_time, int tpqn,int bpm, const std::string& midiFilePath);
    void analyzeMidiEvent(const std::vector<unsigned char>& data, size_t& pos, unsigned char& runningStatus, double& note_on_time, int& tpqn,int &bpm, const std::string& midiFilePath);
    
    void convertMcToC(const std::string& inputFilename, const std::string& outputFilename);
    pair<int, int> sectionBasedHandAssignment(int inst1, int inst2);


    static int zoneOf(int inst);
    static bool isCrossed(int rightInst, int leftInst);
    void checkCross(int& rightHand, int& leftHand, int prevRightNote, int prevLeftNote);
    
    void assignHandsToEvents(const std::string& inputFilename, const std::string& outputFilename);
    void compensateTotalTimeTo4p8(const std::string& inputPath, const std::string& outputPath);
    void addGroove(int bpm, const std::string& inputFilename, const std::string& outputFilename);
    void convertToMeasureFile(const std::string& inputFilename, const std::string& outputFilename, bool startFlag, bool endFlag);

    Hand distanceTimeBasedHandSelection(int instCurrent, int prevRightNote, int prevLeftNote, double prevRightHit, double prevLeftHit);
    void save_to_csv(const std::string& outputCsvPath, double& note_on_time, int drumNote);

    struct Seg {
        double start, end;
        int drum_avg;    // 0~3
        int cymbal_avg;  // 0~3
    };
    struct VelocityEntry {
        double time;
        int instrument;
        int velocity;
    };

    bool loadSegments(const string& intensityFile, vector<Functions::Seg>& segs); 
    bool applyIntensityToScore(const vector<Functions::Seg>& segs, const string& scoreIn, const string& scoreOut, bool mapTo357);
    void analyzeVelocityWithLowPassFilter(const std::string& velocityFile, const std::string& outputFile);

    void clear_directory(const std::filesystem::path& dir_path);
    
private:
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;   // data 기록하기 위해 필요
};