#pragma once

struct Constants
{
    static constexpr double wheelDiameter = 4;
    static constexpr double pi = 3.14159265359;
    static constexpr double encoderTicksPerRotation = 4096;
    
    static constexpr double wheelCircumference = pi * wheelDiameter;

    //Multiply encoder ticks by this to convert to inches
    static constexpr double inchesOverEncoderTicks = wheelCircumference / encoderTicksPerRotation;

    //Multiply inches by this to convert to encoder ticks
    static constexpr double encoderTicksOverInches = encoderTicksPerRotation / wheelCircumference;

    static constexpr int driveTolerance = 200;
};