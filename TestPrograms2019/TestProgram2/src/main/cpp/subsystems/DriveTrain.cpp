/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include "RobotMap.h"
#include "commands/MecanumDrive.h"
#include "Robot.h"

DriveTrain::DriveTrain() : Subsystem("ExampleSubsystem") {
  
}

void DriveTrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  SetDefaultCommand(new MecanumDrive());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

double changeSpeedFast(double speed){
  double changedSpeed = 0.8*pow(speed,2);
  if(speed > 0) {
    return changedSpeed;
  } 
  else {
    return -1*changedSpeed;
  }
}

double changeSpeedSlow(double speed){
  double changedSpeed = 0.5*pow(speed,2);
  if(speed > 0) {
    return changedSpeed;
  } 
  else {
    return -1*changedSpeed;
  }
}


void DriveTrain::mecanumWheelDrive(double ySpeed, double xSpeed, double zRotation)
{
  if (Robot::f310.getButton(Robot::f310.right_shoulder_button))
  {
    mecanumDrive.DriveCartesian(changeSpeedSlow(ySpeed), changeSpeedSlow(xSpeed), changeSpeedSlow(zRotation));
  }
  else 
  {
    mecanumDrive.DriveCartesian(changeSpeedFast(ySpeed), changeSpeedFast(xSpeed), changeSpeedFast(zRotation));
  }
}
