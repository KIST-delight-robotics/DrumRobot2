#pragma once

#include <atomic>

enum class Main
{
    SystemInit,
    Ideal,
    Test,
    Play,
    Shutdown,
    Pause,
    AddStance,
    Error
};

enum class PlaySub
{
    TimeCheck,
    GenerateTrajectory,
    SolveIK,
    SetCANFrame,
    // SetMaxonCANFrame,
    SendCANFrame,
    // SendMaxonCANFrame,
    ReadMusicSheet
};

enum class AddStanceSub
{
    TimeCheck,
    CheckCommand,
    CheckBuf,
    // CheckMaxonBuf,
    FillBuf,
    SetCANFrame,
    // SetMaxonCANFrame,
    SendCANFrame,
    // SendMaxonCANFrame
};

enum class ReadSub
{
    TimeCheck,
    ReadCANFrame,
    UpdateMotorInfo
};

enum class TestSub
{
    SelectParamByUser,
    SetQValue,
    SetXYZ,
    TestMaxon,
    FillBuf,
    CheckBuf,
    TimeCheck,
    SetCANFrame,
    SetMaxonCANFrame,
    SendCANFrame,
    SendMaxonCANFrame,
    Done
};



struct State
{
    std::atomic<Main> main;
    std::atomic<PlaySub> play;
    std::atomic<AddStanceSub> addstance;
    std::atomic<ReadSub> read;
    std::atomic<TestSub> test;

    State() : main(Main::SystemInit),
              play(PlaySub::ReadMusicSheet),
              addstance(AddStanceSub::CheckCommand),
              read(ReadSub::TimeCheck),
              test(TestSub::SelectParamByUser)
    {
    }
};
