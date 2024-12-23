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
    SendCANFrame,
    ReadMusicSheet
};

enum class AddStanceSub
{
    TimeCheck,
    CheckCommand,
    CheckBuf,
    FillBuf,
    SetCANFrame,
    SendCANFrame
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
    FillBuf,
    CheckBuf,
    TimeCheck,
    SetCANFrame,
    SendCANFrame,
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
              play(PlaySub::TimeCheck),
              addstance(AddStanceSub::CheckCommand),
              read(ReadSub::TimeCheck),
              test(TestSub::SelectParamByUser)
    {
    }
};
